# Головний файл запуску GUI для керування двома дронами.
# Тут обробляються аргументи командного рядка, перевіряються маршрути,
# створюються Qt-вікно, панелі дронів, MAVLink-воркери та відеоворкери Gazebo.
import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

from drone_client import load_route


# Зчитує аргументи командного рядка для двох дронів.
# Повертає argparse.Namespace з MAVLink портами, файлами маршрутів,
# висотами зльоту та опційними Gazebo video topics.
def parse_args():
    parser = argparse.ArgumentParser(description="Two drone control GUI")
    # Кожен дрон має власний MAVLink endpoint, маршрут і опційний відеотопік.
    parser.add_argument(
        "--port1",
        required=True,
        help="Drone 1 MAVLink connection string, for example udpin:0.0.0.0:14550",
    )
    parser.add_argument(
        "--route1",
        required=True,
        help="Path to Drone 1 route JSON file",
    )
    parser.add_argument(
        "--takeoff-alt1",
        type=float,
        default=None,
        help="Drone 1 takeoff altitude. If omitted, route takeoff_alt or first point alt is used.",
    )
    parser.add_argument(
        "--video_topic1",
        "--video-topic1",
        default=None,
        help="Gazebo transport camera topic for Drone 1 video feed",
    )
    parser.add_argument(
        "--port2",
        required=True,
        help="Drone 2 MAVLink connection string, for example udpin:0.0.0.0:14551",
    )
    parser.add_argument(
        "--route2",
        required=True,
        help="Path to Drone 2 route JSON file",
    )
    parser.add_argument(
        "--takeoff-alt2",
        type=float,
        default=None,
        help="Drone 2 takeoff altitude. If omitted, route takeoff_alt or first point alt is used.",
    )
    parser.add_argument(
        "--video_topic2",
        "--video-topic2",
        default=None,
        help="Gazebo transport camera topic for Drone 2 video feed",
    )
    parser.add_argument(
        "--video-fps",
        type=int,
        default=20,
        help="Maximum GUI video refresh rate in frames per second",
    )
    # Прапорець дозволяє запускати GUI без підписки на відео навіть за наявності topic.
    parser.add_argument(
        "--disable-video",
        action="store_true",
        help="Disable Gazebo video subscriptions even when video topics are provided",
    )
    return parser.parse_args()


# Перевіряє, що файл маршруту існує, є звичайним файлом,
# і завантажує його через load_route().
# route_arg - шлях з CLI; label - назва дрона для зрозумілих повідомлень про помилки.
# Повертає словник маршруту.
def load_checked_route(route_arg, label):
    route_path = Path(route_arg)
    if not route_path.exists():
        raise ValueError(f"{label} route file does not exist: {route_path}")
    if not route_path.is_file():
        raise ValueError(f"{label} route path is not a file: {route_path}")

    return load_route(route_path)


# Перевіряє взаємну коректність аргументів командного рядка.
# args - результат parse_args().
# Повертає два завантажені маршрути: для Drone 1 і Drone 2.
def validate_args(args):
    route1 = load_checked_route(args.route1, "Drone 1")
    route2 = load_checked_route(args.route2, "Drone 2")

    # Висота зльоту має бути додатною, якщо користувач задав її вручну.
    if args.takeoff_alt1 is not None and args.takeoff_alt1 <= 0:
        raise ValueError("--takeoff-alt1 must be greater than 0")
    if args.takeoff_alt2 is not None and args.takeoff_alt2 <= 0:
        raise ValueError("--takeoff-alt2 must be greater than 0")

    # Два дрони не можуть слухати один і той самий MAVLink endpoint.
    if args.port1 == args.port2:
        raise ValueError("--port1 and --port2 must use different MAVLink endpoints")

    # Обмеження FPS захищає GUI від надто частих оновлень кадрів.
    if args.video_fps < 1 or args.video_fps > 60:
        raise ValueError("--video-fps must be between 1 and 60")

    return route1, route2


# Перевіряє, чи можна використовувати Gazebo video topic до запуску воркера.
# topic - назва Gazebo transport topic; timeout_sec - короткий таймаут для gz topic -l.
# Повертає пару (is_valid, message): True/"" для успіху або False/причина помилки.
def validate_video_topic(topic, timeout_sec=5.0):
    if topic is None or not topic.strip():
        return False, "Video topic is not provided"

    # Якщо CLI Gazebo недоступний, не зупиняємо GUI, а просто вимикаємо відео.
    if shutil.which("gz") is None:
        return False, "Video unavailable: gz command was not found in PATH"

    normalized_topic = topic.strip().lstrip("/")

    try:
        # Список topic береться окремим процесом, щоб не блокувати Qt event loop.
        result = subprocess.run(
            ["gz", "topic", "-l"],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
    except subprocess.TimeoutExpired:
        return False, f"Video unavailable: gz topic list timed out after {timeout_sec:.1f}s"
    except OSError as exc:
        return False, f"Video unavailable: failed to run gz topic list: {exc}"

    if result.returncode != 0:
        detail = (result.stderr or result.stdout or "").strip()
        if detail:
            detail = detail.splitlines()[0]
            return False, f"Video unavailable: gz topic list failed: {detail}"
        return False, f"Video unavailable: gz topic list failed with exit code {result.returncode}"

    topics = {line.strip().lstrip("/") for line in result.stdout.splitlines() if line.strip()}
    if normalized_topic not in topics:
        # Якщо запитаний topic не знайдений, показуємо близькі image/camera topics
        # для швидшого вибору правильного Gazebo camera topic.
        image_topics = sorted(
            topic
            for topic in topics
            if "image" in topic.lower() or "camera" in topic.lower()
        )
        if image_topics:
            examples = ", ".join("/" + topic for topic in image_topics[:5])
            return (
                False,
                f"Video topic not found: {topic.strip()} "
                f"(available image/camera topic(s): {examples})",
            )
        if topics:
            examples = ", ".join("/" + topic for topic in sorted(topics)[:3])
            return (
                False,
                f"Video topic not found: {topic.strip()} (found {len(topics)} topic(s), e.g. {examples})",
            )
        return False, f"Video topic not found: {topic.strip()} (Gazebo reported no topics)"

    return True, ""


# У WSLg інколи примусове QT_QPA_PLATFORM=xcb заважає Wayland.
# Функція прибирає це значення, щоб Qt міг сам обрати правильну платформу.
def normalize_qt_platform_for_wslg():
    if os.environ.get("QT_QPA_PLATFORM") != "xcb":
        return

    if not os.environ.get("WAYLAND_DISPLAY"):
        return

    print(
        "Warning: QT_QPA_PLATFORM=xcb is forced while WSLg Wayland is available. "
        "Using Qt default platform detection instead.",
        file=sys.stderr,
    )
    os.environ.pop("QT_QPA_PLATFORM", None)


# Точка входу програми: перевіряє аргументи, створює QApplication і головне вікно.
# Повертає код завершення Qt application loop.
def main():
    try:
        args = parse_args()
        route1, route2 = validate_args(args)
    except Exception as exc:
        print(f"Input error: {exc}", file=sys.stderr)
        return 2

    normalize_qt_platform_for_wslg()

    try:
        # Qt імпортується після перевірки CLI, щоб помилки вводу не вимагали GUI-залежностей.
        from PySide6.QtCore import QThread
        from PySide6.QtWidgets import QApplication, QHBoxLayout, QMainWindow, QWidget

        from drone_worker import DroneWorker
        from video_worker import VideoWorker, load_gazebo_video_bindings, normalize_gazebo_topic
        from widgets.drone_panel import DronePanel
    except ImportError as exc:
        print(f"Dependency error: {exc}", file=sys.stderr)
        return 2

    # Головне вікно тримає дві незалежні runtime-структури:
    # окремий DroneWorker/QThread і, за потреби, VideoWorker/QThread для кожного дрона.
    class MainWindow(QMainWindow):
        # drone_configs - список конфігурацій для панелей Drone 1 і Drone 2.
        def __init__(self, drone_configs):
            super().__init__()
            self.setWindowTitle("Two Drone Control GUI")
            self.drone_runtimes = []
            self.closing = False

            central = QWidget()
            layout = QHBoxLayout(central)

            for config in drone_configs:
                # Для кожного дрона створюється окрема панель, MAVLink worker і video worker.
                runtime = self._create_drone_runtime(config)
                self.drone_runtimes.append(runtime)
                layout.addWidget(runtime["panel"])

            self.setCentralWidget(central)

            for runtime in self.drone_runtimes:
                # MAVLink polling запускається у власному QThread, щоб не блокувати GUI.
                runtime["thread"].start()

        # Створює всі об'єкти, потрібні одному дрону: GUI-панель, MAVLink worker,
        # опційний video worker і зв'язки Qt signals/slots між ними.
        # config - словник з port, route, takeoff_alt, video_topic тощо.
        # Повертає runtime-словник, який зберігається до завершення програми.
        def _create_drone_runtime(self, config):
            thread = QThread(self)
            worker = DroneWorker(
                port=config["port"],
                route=config["route"],
                takeoff_alt=config["takeoff_alt"],
                land_after_route=True,
            )
            worker.moveToThread(thread)
            # started -> start_telemetry_polling запускає QTimer вже у worker thread.
            thread.started.connect(worker.start_telemetry_polling)
            thread.finished.connect(worker.deleteLater)

            panel = DronePanel(config["title"], config["route"])

            # Кнопки GUI надсилають сигнали у worker; прямого MAVLink-коду у віджеті немає.
            panel.connect_requested.connect(worker.connect_to_drone)
            panel.start_requested.connect(panel.set_mission_running)
            panel.start_requested.connect(worker.start_mission)

            # Worker повертає телеметрію, статус, лог і позицію через Qt-сигнали.
            worker.connected.connect(panel.set_connected)
            worker.telemetry_updated.connect(panel.update_telemetry)
            worker.log_message.connect(panel.add_log)
            worker.position_updated.connect(panel.update_position)
            worker.mission_status.connect(panel.set_status)
            worker.error.connect(lambda message, p=panel: self._handle_error(p, message))
            worker.finished.connect(panel.set_mission_finished)

            video_thread = None
            video_worker = None
            video_topic = config["video_topic"]
            if config["video_enabled"]:
                # Відео перевіряється до створення worker, щоб відсутній Gazebo topic
                # не ламав запуск MAVLink та основного GUI.
                is_valid_video_topic, video_message = self._validate_video_startup(video_topic)
            else:
                is_valid_video_topic = False
                video_message = "Video topic is not provided"

            if not is_valid_video_topic:
                self._handle_video_status(panel, video_message)

            if config["video_enabled"] and is_valid_video_topic:
                video_thread = QThread(self)
                video_worker = VideoWorker(normalize_gazebo_topic(video_topic), fps=config["video_fps"])
                video_worker.moveToThread(video_thread)
                # Відеопотік має окремий QThread: декодування кадрів не блокує Qt UI.
                video_thread.started.connect(video_worker.start)
                video_thread.finished.connect(video_worker.deleteLater)
                video_worker.frame_ready.connect(panel.set_video_frame)
                video_worker.status_changed.connect(
                    lambda message, p=panel: self._handle_video_status(p, message)
                )
                video_worker.error.connect(lambda message, p=panel: self._handle_video_error(p, message))
                video_thread.start()

            return {
                "thread": thread,
                "worker": worker,
                "panel": panel,
                "video_thread": video_thread,
                "video_worker": video_worker,
            }

        def _handle_error(self, panel, message):
            panel.set_status("Error")
            panel.add_log(f"ERROR: {message}")

        # Записує помилки/діагностику відео саме у лог відповідної панелі дрона.
        def _handle_video_error(self, panel, message):
            panel.add_log(f"VIDEO: {message}")

        # Перевіряє Gazebo topic і Python bindings до запуску VideoWorker.
        # video_topic - topic з CLI; повертає (is_valid, message).
        def _validate_video_startup(self, video_topic):
            is_valid_video_topic, video_message = validate_video_topic(video_topic)
            if not is_valid_video_topic:
                return is_valid_video_topic, video_message

            Image, Node, binding_error = load_gazebo_video_bindings()
            if Image is None or Node is None:
                print(
                    f"Warning: Gazebo Python transport is unavailable: {binding_error}",
                    file=sys.stderr,
                )
                return False, "Gazebo Python transport is unavailable"

            return True, ""

        # Оновлює текстовий статус у video panel і не дублює однакові повідомлення у лог.
        def _handle_video_status(self, panel, message):
            panel.set_video_status(message)

            if not message:
                return

            if getattr(panel, "_last_logged_video_status", None) == message:
                return

            panel._last_logged_video_status = message
            panel.add_log(f"VIDEO: {message}")

        # Коректно зупиняє MAVLink і video workers під час закриття GUI.
        # Спочатку просимо worker зупинитись, потім завершуємо QThread.
        def stop_all(self):
            if self.closing:
                return

            self.closing = True
            for runtime in self.drone_runtimes:
                runtime["worker"].request_stop()
                if runtime["video_worker"] is not None:
                    runtime["video_worker"].request_stop()

            for runtime in self.drone_runtimes:
                thread = runtime["thread"]
                thread.quit()
                if not thread.wait(3000):
                    thread.terminate()
                    thread.wait(1000)

                video_thread = runtime["video_thread"]
                if video_thread is not None:
                    video_thread.quit()
                    if not video_thread.wait(3000):
                        video_thread.terminate()
                        video_thread.wait(1000)

        def closeEvent(self, event):
            self.stop_all()
            super().closeEvent(event)

    app = QApplication(sys.argv)
    # Конфігурації розділені явно, щоб Drone 1 і Drone 2 мали незалежні порти,
    # маршрути, відеотопіки та worker-потоки.
    drone_configs = [
        {
            "title": "DRONE 1",
            "port": args.port1,
            "route": route1,
            "takeoff_alt": args.takeoff_alt1,
            "video_topic": args.video_topic1,
            "video_fps": args.video_fps,
            "video_enabled": not args.disable_video,
        },
        {
            "title": "DRONE 2",
            "port": args.port2,
            "route": route2,
            "takeoff_alt": args.takeoff_alt2,
            "video_topic": args.video_topic2,
            "video_fps": args.video_fps,
            "video_enabled": not args.disable_video,
        },
    ]
    window = MainWindow(drone_configs)
    app.aboutToQuit.connect(window.stop_all)
    window.resize(1600, 850)
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
