# Низькорівневий MAVLink-клієнт для одного дрона.
# Файл містить допоміжні функції для маршруту/координат і клас DroneClient,
# який відповідає за підключення, читання телеметрії та надсилання команд польоту.
from pymavlink import mavutil
import json
import math
import time


EARTH_RADIUS_M = 6378137


# Створює початкову структуру телеметрії з порожніми значеннями.
# Повертає словник, який далі оновлюється MAVLink-повідомленнями.
def empty_telemetry():
    return {
        "mode": None,
        "armed": None,
        "battery": None,
        "attitude": None,
        "position": None,
        "local": None,
        "imu": None,
    }


# Обчислює приблизну відстань між двома GPS-точками у метрах.
# lat1/lon1 - поточна позиція, lat2/lon2 - цільова позиція.
# Використовується для перевірки досягнення waypoint.
def distance_m(lat1, lon1, lat2, lon2):
    r = 6371000
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)

    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return r * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# Перетворює GPS-координати у локальні ENU-координати для карти траєкторії.
# lat/lon - точка, яку перетворюємо; lat0/lon0 - початкова опорна точка маршруту.
# Повертає (east, north) у метрах.
def gps_to_enu(lat, lon, lat0, lon0):
    d_lat = (lat - lat0) * math.pi / 180
    d_lon = (lon - lon0) * math.pi / 180

    north = d_lat * EARTH_RADIUS_M
    east = d_lon * EARTH_RADIUS_M * math.cos(lat0 * math.pi / 180)
    return east, north


# Завантажує JSON-маршрут і перевіряє, що кожна точка має lat/lon/alt.
# path - шлях до route JSON; повертає словник маршруту.
def load_route(path):
    with open(path, "r", encoding="utf-8") as file:
        route = json.load(file)

    points = route.get("points")
    if not points:
        raise ValueError("JSON route must contain non-empty 'points' list")

    for index, point in enumerate(points, start=1):
        for key in ("lat", "lon", "alt"):
            if key not in point:
                raise ValueError(f"Route point {index} is missing '{key}'")

    return route


# Клас інкапсулює одну MAVLink-сесію з дроном.
# Він не знає про GUI напряму: повідомлення передаються через logger і telemetry_callback.
class DroneClient:
    # port - MAVLink connection string; logger - функція для логів;
    # telemetry_callback - функція, яку викликаємо після оновлення телеметрії.
    def __init__(self, port, logger=None, telemetry_callback=None):
        self.port = port
        self.mav = None
        self.telemetry = empty_telemetry()
        self.logger = logger
        self.telemetry_callback = telemetry_callback

    # Єдина точка логування для CLI-режиму і GUI-режиму.
    # Якщо logger не передано, повідомлення друкуються у stdout.
    def log(self, message):
        if self.logger is not None:
            self.logger(message)
        else:
            print(message)

    # Відкриває MAVLink-з'єднання і чекає heartbeat від автопілота.
    # heartbeat_timeout - максимальний час очікування heartbeat.
    # Повертає об'єкт mavlink_connection або кидає TimeoutError.
    def connect(self, heartbeat_timeout=30):
        self.log(f"Connecting to {self.port}...")
        self.mav = mavutil.mavlink_connection(self.port)

        # Heartbeat підтверджує, що за endpoint є живий автопілот/SITL.
        self.log("Waiting for heartbeat...")
        msg = self.mav.wait_heartbeat(timeout=heartbeat_timeout)
        if msg is None:
            self.close()
            raise TimeoutError(f"No heartbeat received within {heartbeat_timeout} seconds")

        self.log(
            f"Heartbeat OK: system={self.mav.target_system}, "
            f"component={self.mav.target_component}"
        )
        self.request_telemetry_streams()
        return self.mav

    # Закриває MAVLink-з'єднання, якщо воно було відкрите.
    # Метод безпечний для повторного виклику під час зупинки worker.
    def close(self):
        if self.mav is None:
            return

        try:
            close = getattr(self.mav, "close", None)
            if close is not None:
                close()
            elif getattr(self.mav, "port", None) is not None:
                self.mav.port.close()
        except Exception as exc:
            self.log(f"Error while closing MAVLink connection: {exc}")
        finally:
            self.mav = None

    # Просить автопілот надсилати конкретний MAVLink message_id з частотою hz.
    # Використовується для керування потоком телеметрії, щоб GUI отримував потрібні дані.
    def request_message_interval(self, message_id, hz):
        interval_us = int(1_000_000 / hz)

        # MAV_CMD_SET_MESSAGE_INTERVAL задає інтервал публікації повідомлення у мікросекундах.
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0, 0, 0, 0, 0
        )

    # Налаштовує набір MAVLink-повідомлень, потрібних GUI:
    # heartbeat, батарея, орієнтація, GPS-позиція, локальна позиція та IMU.
    def request_telemetry_streams(self):
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 5)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, 5)

    # Тонка обгортка над pymavlink recv_match, щоб інший код не звертався напряму до self.mav.
    def recv_match(self, *args, **kwargs):
        return self.mav.recv_match(*args, **kwargs)

    # Читає одне MAVLink-повідомлення телеметрії і одразу оновлює self.telemetry.
    # timeout=0 означає неблокуюче читання; повертає msg або None.
    def read_telemetry_message(self, timeout=0):
        if self.mav is None:
            return None

        msg = self.mav.recv_match(blocking=timeout > 0, timeout=timeout)
        if msg is not None:
            self.update_telemetry(msg)
        return msg

    # Розбирає різні типи MAVLink-повідомлень і оновлює загальний словник telemetry.
    # msg - MAVLink-повідомлення від pymavlink; повертає True, якщо повідомлення оброблене.
    def update_telemetry(self, msg):
        if msg is None:
            return False

        msg_type = msg.get_type()

        if msg_type == "HEARTBEAT":
            # HEARTBEAT дає поточний режим польоту і прапорець armed/disarmed.
            self.telemetry["mode"] = mavutil.mode_string_v10(msg)
            self.telemetry["armed"] = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )

        elif msg_type == "SYS_STATUS":
            # Значення батареї в MAVLink приходять у мілівольтах/сантіамперах.
            self.telemetry["battery"] = {
                "voltage_v": msg.voltage_battery / 1000.0,
                "current_a": msg.current_battery / 100.0,
                "remaining_percent": msg.battery_remaining,
            }

        elif msg_type == "ATTITUDE":
            # Кути орієнтації потрібні для відображення roll/pitch/yaw у GUI.
            self.telemetry["attitude"] = {
                "roll": msg.roll,
                "pitch": msg.pitch,
                "yaw": msg.yaw,
                "rollspeed": msg.rollspeed,
                "pitchspeed": msg.pitchspeed,
                "yawspeed": msg.yawspeed,
            }

        elif msg_type == "GLOBAL_POSITION_INT":
            # GLOBAL_POSITION_INT кодує lat/lon у 1e7, altitude у міліметрах,
            # швидкості у см/с, тому тут усе переводиться у зручні одиниці.
            self.telemetry["position"] = {
                "lat": msg.lat / 1e7,
                "lon": msg.lon / 1e7,
                "relative_alt_m": msg.relative_alt / 1000.0,
                "vx": msg.vx / 100.0,
                "vy": msg.vy / 100.0,
                "vz": msg.vz / 100.0,
            }

        elif msg_type == "LOCAL_POSITION_NED":
            # Локальна NED-позиція зберігається для діагностики, хоча карта використовує GPS->ENU.
            self.telemetry["local"] = {
                "x": msg.x,
                "y": msg.y,
                "z": msg.z,
                "vx": msg.vx,
                "vy": msg.vy,
                "vz": msg.vz,
            }

        elif msg_type == "RAW_IMU":
            # Сирі IMU-дані показуються у телеметрії для перевірки роботи сенсорів.
            self.telemetry["imu"] = {
                "xacc": msg.xacc,
                "yacc": msg.yacc,
                "zacc": msg.zacc,
                "xgyro": msg.xgyro,
                "ygyro": msg.ygyro,
                "zgyro": msg.zgyro,
                "xmag": msg.xmag,
                "ymag": msg.ymag,
                "zmag": msg.zmag,
            }

        if self.telemetry_callback is not None:
            # Копія словника віддається callback, щоб GUI отримував актуальний snapshot.
            self.telemetry_callback(dict(self.telemetry))

        return True

    # Друкує поточний snapshot телеметрії у лог.
    # Використовується у CLI-скрипті і під час місії для періодичних повідомлень.
    def print_telemetry(self):
        self.log("")
        self.log(f"Mode: {self.telemetry['mode']}")
        self.log(f"Armed: {self.telemetry['armed']}")
        self.log(f"Battery: {self.telemetry['battery']}")
        self.log(f"Attitude: {self.telemetry['attitude']}")
        self.log(f"Position: {self.telemetry['position']}")
        self.log(f"Local: {self.telemetry['local']}")
        self.log(f"IMU: {self.telemetry['imu']}")

    # Перемикає автопілот у заданий режим, наприклад GUIDED.
    # mode - назва режиму ArduPilot; timeout - час очікування підтвердження.
    # Повертає True після підтвердження або кидає TimeoutError.
    def set_mode(self, mode, timeout=15):
        mode_id = self.mav.mode_mapping()[mode]
        self.mav.set_mode(mode_id)

        start = time.time()

        while time.time() - start < timeout:
            # Підтвердження режиму приходить через HEARTBEAT, тому чекаємо саме його.
            msg = self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg is None:
                continue

            self.update_telemetry(msg)

            current_mode = mavutil.mode_string_v10(msg)
            if current_mode == mode:
                self.log(f"Mode changed to {mode}")
                return True

        raise TimeoutError(f"Drone did not switch to {mode} mode")

    # Чекає, поки дрон набере потрібну відносну висоту.
    # target_alt - цільова висота у метрах; tolerance - допустима похибка.
    # Повертає True або кидає TimeoutError.
    def wait_until_altitude(self, target_alt, tolerance=0.7, timeout=30):
        self.log(f"Waiting until altitude {target_alt} m...")

        start = time.time()
        last_print = 0

        while time.time() - start < timeout:
            msg = self.mav.recv_match(blocking=True, timeout=1)

            if msg is None:
                self.log("No message while waiting altitude...")
                continue

            self.update_telemetry(msg)

            if self.telemetry["position"] is None:
                continue

            current_alt = self.telemetry["position"]["relative_alt_m"]

            now = time.time()

            # Лог обмежений до 1 разу на секунду, щоб не засмічувати GUI.
            if now - last_print >= 1:
                self.log(f"Current altitude: {current_alt:.2f} m")
                last_print = now

            if abs(current_alt - target_alt) <= tolerance:
                self.log("Target altitude reached")
                return True

        raise TimeoutError("Drone did not reach target altitude")

    # Надсилає команду ARM і чекає, поки HEARTBEAT підтвердить armed=True.
    # timeout - максимальний час очікування; повертає True або кидає TimeoutError.
    def arm(self, timeout=15):
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )

        self.log("Arm command sent, waiting...")

        start = time.time()

        while time.time() - start < timeout:
            msg = self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)

            if msg is None:
                continue

            self.update_telemetry(msg)

            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

            if armed:
                self.log("Drone armed")
                return True

        raise TimeoutError("Drone did not arm. Check pre-arm errors in SITL/MAVProxy.")

    # Надсилає MAV_CMD_NAV_TAKEOFF на задану висоту alt у метрах.
    def takeoff(self, alt):
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0,
            alt
        )
        self.log(f"Takeoff command sent: {alt} m")

    # Надсилає цільову GPS-точку для польоту у GUIDED режимі.
    # lat/lon - GPS координати, alt - відносна висота у метрах.
    def goto_point(self, lat, lon, alt):
        self.mav.mav.set_position_target_global_int_send(
            0,
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            int(0b110111111000),
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        self.log(f"Target point sent: lat={lat}, lon={lon}, alt={alt}")

    # Надсилає команду посадки MAV_CMD_NAV_LAND.
    # Подальше очікування фактичної посадки виконується у DroneWorker.
    def land(self):
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0,
            0, 0, 0
        )
        self.log("Land command sent")
