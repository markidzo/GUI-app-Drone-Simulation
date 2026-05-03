# Qt worker для одного дрона.
# Цей файл з'єднує низькорівневий DroneClient з GUI через сигнали PySide6,
# щоб MAVLink-підключення, телеметрія і місія не блокували головний UI-потік.
import time

from PySide6.QtCore import QObject, QTimer, Signal, Slot

from drone_client import DroneClient, distance_m, gps_to_enu


# DroneWorker живе у власному QThread для кожного дрона.
# Він приймає команди від GUI, виконує місію через DroneClient і відправляє
# телеметрію/логи/статуси назад у DronePanel через Qt signals.
class DroneWorker(QObject):
    # Сигнали безпечно передають дані з worker thread у GUI thread.
    connected = Signal()
    telemetry_updated = Signal(object)
    log_message = Signal(str)
    position_updated = Signal(float, float)
    mission_status = Signal(str)
    error = Signal(str)
    finished = Signal()

    # port - MAVLink endpoint; route - словник маршруту; takeoff_alt - опційна висота зльоту;
    # land_after_route визначає, чи потрібно автоматично сідати після завершення маршруту.
    def __init__(self, port, route, takeoff_alt=None, land_after_route=True):
        super().__init__()
        self.port = port
        self.route = route
        self.takeoff_alt = takeoff_alt
        self.land_after_route = land_after_route
        self.client = None
        self.stop_requested = False
        self.mission_running = False
        self.telemetry_timer = None

        first_point = route["points"][0]
        self.ref_lat = first_point["lat"]
        self.ref_lon = first_point["lon"]

    # Відправляє текстовий лог у GUI-панель через сигнал.
    def _log(self, message):
        self.log_message.emit(str(message))

    # Callback для DroneClient: передає telemetry у GUI і оновлює позицію на карті.
    # telemetry - словник з останніми MAVLink-даними.
    def _handle_telemetry(self, telemetry):
        self.telemetry_updated.emit(telemetry)

        position = telemetry.get("position")
        if not position:
            return

        # Карта очікує локальні координати East/North відносно першої точки маршруту.
        east, north = gps_to_enu(
            position["lat"],
            position["lon"],
            self.ref_lat,
            self.ref_lon,
        )
        self.position_updated.emit(east, north)

    # Ліниво створює DroneClient, коли користувач підключається або стартує місію.
    # Повертає готовий екземпляр DroneClient.
    def _ensure_client(self):
        if self.client is None:
            self.client = DroneClient(
                self.port,
                logger=self._log,
                telemetry_callback=self._handle_telemetry,
            )
        return self.client

    # Просить worker зупинитися і закриває MAVLink-з'єднання.
    # Викликається під час закриття GUI або зупинки потоків.
    def request_stop(self):
        self.stop_requested = True
        if self.client is not None:
            self.client.close()

    # Запускає таймер періодичного читання телеметрії у worker thread.
    # Slot викликається після старту QThread.
    @Slot()
    def start_telemetry_polling(self):
        if self.telemetry_timer is not None:
            return

        # QTimer працює у потоці worker і не блокує головний Qt event loop.
        self.telemetry_timer = QTimer(self)
        self.telemetry_timer.setInterval(100)
        self.telemetry_timer.timeout.connect(self.poll_telemetry)
        self.telemetry_timer.start()

    # Неблокуюче зчитування накопичених MAVLink-повідомлень у idle-режимі.
    # Під час активної місії телеметрію читає основний mission loop.
    @Slot()
    def poll_telemetry(self):
        if self.mission_running or self.stop_requested:
            return

        client = self.client
        if client is None or client.mav is None:
            return

        # Обмеження у 10 повідомлень за тік захищає worker від нескінченного циклу,
        # якщо повідомлень накопичилося багато.
        for _ in range(10):
            msg = client.read_telemetry_message(timeout=0)
            if msg is None:
                break

    # Підключає DroneClient до MAVLink endpoint і запускає телеметричні стріми.
    # Slot під'єднаний до кнопки Connect у DronePanel.
    @Slot()
    def connect_to_drone(self):
        try:
            self.stop_requested = False
            client = self._ensure_client()
            if client.mav is None:
                client.connect()
            client.request_telemetry_streams()
            self.connected.emit()
            self.mission_status.emit("Connected")
        except Exception as exc:
            self.error.emit(str(exc))

    # Виконує повну місію: connect, GUIDED, arm, takeoff, waypoint loop, landing.
    # Slot під'єднаний до кнопки Start Mission у DronePanel.
    @Slot()
    def start_mission(self):
        if self.mission_running:
            return

        self.mission_running = True
        self.stop_requested = False

        try:
            client = self._ensure_client()
            if client.mav is None:
                client.connect()
                self.connected.emit()
            client.request_telemetry_streams()

            points = self.route["points"]
            takeoff_alt = self.takeoff_alt
            if takeoff_alt is None:
                # Якщо CLI не задав висоту, беремо takeoff_alt з маршруту або altitude першої точки.
                takeoff_alt = self.route.get("takeoff_alt", points[0]["alt"])

            reach_radius_m = self.route.get("reach_radius_m", 1.5)
            point_timeout = self.route.get("point_timeout_sec", 60)
            alt_tolerance_m = self.route.get("alt_tolerance_m", 1.0)

            self.mission_status.emit("Setting GUIDED mode")
            client.set_mode("GUIDED")
            if self.stop_requested:
                return

            # Перед зльотом ArduPilot має бути armed; помилки pre-arm підуть у виняток/лог.
            self.mission_status.emit("Arming")
            client.arm()
            if self.stop_requested:
                return

            self.mission_status.emit("Taking off")
            client.takeoff(takeoff_alt)
            client.wait_until_altitude(takeoff_alt)
            if self.stop_requested:
                return

            last_print = 0
            last_current_print = 0

            for index, point in enumerate(points):
                if self.stop_requested:
                    return

                # Кожна точка маршруту має lat/lon/alt, перевірені під час load_route().
                target_lat = point["lat"]
                target_lon = point["lon"]
                target_alt = point["alt"]

                self._log("")
                self._log(f"Going to point {index + 1}/{len(points)}")
                self.mission_status.emit(f"Going to point {index + 1}/{len(points)}")
                client.goto_point(target_lat, target_lon, target_alt)

                point_start = time.time()
                last_print = 0

                while True:
                    if self.stop_requested:
                        return

                    # У mission loop читаємо MAVLink блокуюче, бо саме тут очікуємо рух до точки.
                    msg = client.recv_match(blocking=True, timeout=2)

                    if msg is None:
                        self._log("No message...")
                        continue

                    client.update_telemetry(msg)

                    now = time.time()

                    if now - point_start > point_timeout:
                        self._log(
                            f"Timeout: point {index + 1} was not reached "
                            f"in {point_timeout} seconds"
                        )
                        break

                    # Раз на секунду друкуємо повну телеметрію, щоб лог залишався читабельним.
                    if now - last_print >= 1:
                        client.print_telemetry()
                        last_print = now

                    if client.telemetry["position"] is None:
                        continue

                    current_lat = client.telemetry["position"]["lat"]
                    current_lon = client.telemetry["position"]["lon"]
                    current_alt = client.telemetry["position"]["relative_alt_m"]

                    dist = distance_m(
                        current_lat,
                        current_lon,
                        target_lat,
                        target_lon,
                    )

                    alt_error = abs(current_alt - target_alt)

                    # Окремий короткий лог показує прогрес до поточного waypoint.
                    if now - last_current_print >= 1:
                        self._log(
                            f"Current: lat={current_lat:.7f}, lon={current_lon:.7f}, "
                            f"alt={current_alt:.2f} m | "
                            f"distance to point {index + 1}/{len(points)}={dist:.2f} m | "
                            f"alt_error={alt_error:.2f} m"
                        )
                        last_current_print = now

                    # Waypoint вважається досягнутим лише за відстанню і висотою.
                    if dist < reach_radius_m and alt_error < alt_tolerance_m:
                        self._log(f"Point {index + 1} reached")
                        break

            self._log("")
            self._log("Route completed")
            self.mission_status.emit("Route completed")

            if self.land_after_route:
                client.land()
                self.mission_status.emit("Landing")
                # Посадка не миттєва: окремо чекаємо зменшення висоти і disarm.
                self._wait_until_landed(client)

        except Exception as exc:
            if not self.stop_requested:
                self.error.emit(str(exc))
        finally:
            self.mission_running = False
            if self.stop_requested:
                self.mission_status.emit("Stopped")
            self.finished.emit()

    # Чекає фактичного завершення посадки після MAV_CMD_NAV_LAND.
    # client - активний DroneClient; ground_alt_m - висота, яку вважаємо землею;
    # no_progress_timeout - таймаут без помітного зниження; min_descent_progress_m - поріг прогресу.
    def _wait_until_landed(
        self,
        client,
        ground_alt_m=0.35,
        no_progress_timeout=60,
        min_descent_progress_m=0.2,
    ):
        last_print = 0
        last_progress_time = time.time()
        lowest_altitude = None

        while True:
            if self.stop_requested:
                return

            msg = client.recv_match(blocking=True, timeout=1)
            if msg is None:
                self._log("No telemetry while landing...")
                continue

            client.update_telemetry(msg)
            position = client.telemetry.get("position")
            armed = client.telemetry.get("armed")

            now = time.time()
            if position is not None and now - last_print >= 1:
                self._log(
                    f"Landing: altitude={position['relative_alt_m']:.2f} m, "
                    f"armed={armed}"
                )
                last_print = now

            if position is None:
                continue

            altitude = position["relative_alt_m"]
            if lowest_altitude is None:
                lowest_altitude = altitude
                last_progress_time = now
            elif altitude < lowest_altitude - min_descent_progress_m:
                # Якщо висота зменшується, оновлюємо таймер прогресу посадки.
                lowest_altitude = altitude
                last_progress_time = now

            # Посадка вважається завершеною, коли дрон низько і ArduPilot вже disarmed.
            if altitude <= ground_alt_m and armed is False:
                self._log("Landing complete")
                self.mission_status.emit("Landed")
                return

            # Якщо висота довго не змінюється, не блокуємо GUI назавжди.
            if now - last_progress_time > no_progress_timeout:
                self._log(
                    "Landing wait timed out: no altitude progress detected. "
                    "Mission can be started again if the drone is safe."
                )
                self.mission_status.emit("Landing timeout")
                return
