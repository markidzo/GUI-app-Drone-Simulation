# Простий CLI-скрипт для перегляду MAVLink-телеметрії одного дрона.
# Він не керує місією, а лише підключається, читає повідомлення і друкує
# поточний стан раз на секунду. Корисний для діагностики SITL/MAVLink.
from pymavlink import mavutil
import time
import argparse


# Зчитує MAVLink endpoint з командного рядка.
# Повертає argparse.Namespace з полем port.
def parse_args():
    parser = argparse.ArgumentParser(description="MAVLink telemetry receiver")
    parser.add_argument(
        "--port",
        type=str,
        default="udpin:0.0.0.0:14550",
        help="MAVLink connection string (e.g. udpin:0.0.0.0:14550)"
    )
    return parser.parse_args()


# Основний цикл: підключення до MAVLink, очікування heartbeat і читання телеметрії.
def main():
    args = parse_args()
    port = args.port

    print(f"Connecting to {port}...")
    mav = mavutil.mavlink_connection(port)

    # Heartbeat підтверджує, що автопілот доступний на заданому endpoint.
    print("Waiting for heartbeat...")
    mav.wait_heartbeat()
    print(f"Heartbeat OK: system={mav.target_system}, component={mav.target_component}")

    last_print = time.time()

    telemetry = {
        "mode": None,
        "armed": None,
        "battery": None,
        "attitude": None,
        "position": None,
        "imu": None,
    }

    while True:
        # Блокуюче читання з таймаутом підходить для CLI, бо тут немає GUI event loop.
        msg = mav.recv_match(blocking=True, timeout=2)

        if msg is None:
            print("No message...")
            continue

        msg_type = msg.get_type()

        if msg_type == "HEARTBEAT":
            # HEARTBEAT містить режим польоту і armed/disarmed стан.
            telemetry["mode"] = mavutil.mode_string_v10(msg)
            telemetry["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        elif msg_type == "SYS_STATUS":
            # MAVLink передає батарею у масштабованих одиницях, тому переводимо у V/A/%.
            telemetry["battery"] = {
                "voltage_v": msg.voltage_battery / 1000.0,
                "current_a": msg.current_battery / 100.0,
                "remaining_percent": msg.battery_remaining,
            }

        elif msg_type == "ATTITUDE":
            # Орієнтація потрібна для перевірки стабілізації і руху дрона.
            telemetry["attitude"] = {
                "roll": msg.roll,
                "pitch": msg.pitch,
                "yaw": msg.yaw,
                "rollspeed": msg.rollspeed,
                "pitchspeed": msg.pitchspeed,
                "yawspeed": msg.yawspeed,
            }

        elif msg_type == "GLOBAL_POSITION_INT":
            # GPS координати і швидкості переводяться у людські одиниці.
            telemetry["position"] = {
                "lat": msg.lat / 1e7,
                "lon": msg.lon / 1e7,
                "relative_alt_m": msg.relative_alt / 1000.0,
                "vx": msg.vx / 100.0,
                "vy": msg.vy / 100.0,
                "vz": msg.vz / 100.0,
            }

        elif msg_type == "RAW_IMU":
            # RAW_IMU показує сирі дані акселерометра, гіроскопа і магнітометра.
            telemetry["imu"] = {
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

        if time.time() - last_print >= 1:
            # Друк обмежений до 1 Гц, щоб консоль залишалась читабельною.
            print("\n--- TELEMETRY ---")
            print("Mode:", telemetry["mode"])
            print("Armed:", telemetry["armed"])
            print("Battery:", telemetry["battery"])
            print("Attitude:", telemetry["attitude"])
            print("Position:", telemetry["position"])
            print("IMU:", telemetry["imu"])
            last_print = time.time()


if __name__ == "__main__":
    main()
