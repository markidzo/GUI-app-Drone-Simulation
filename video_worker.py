# Qt worker для одного Gazebo video topic.
# Він підписується на Gazebo Transport image topic, перетворює кадри у QImage
# і передає їх у GUI через Qt signals без блокування головного потоку.
import threading
import time
import zlib

from PySide6.QtCore import QObject, QTimer, Signal, Slot
from PySide6.QtGui import QImage


# Завантажує Python bindings Gazebo Sim 8 / Harmonic.
# Повертає (ImageClass, NodeClass, error): error=None означає успіх.
def load_gazebo_video_bindings():
    try:
        # Для Gazebo Sim 8.11.0 модулі мають версійні імена msgs10 і transport13.
        from gz.msgs10.image_pb2 import Image
        from gz.transport13 import Node
    except Exception as exc:
        return None, None, str(exc)

    return Image, Node, None


# Приводить topic до канонічного вигляду Gazebo з початковим "/".
# topic - рядок з CLI; повертає нормалізований рядок.
def normalize_gazebo_topic(topic):
    normalized = topic.strip()
    if normalized and not normalized.startswith("/"):
        normalized = "/" + normalized
    return normalized


# Робить повідомлення про помилку імпорту Gazebo bindings коротшим для GUI-логу.
def summarize_binding_error(binding_error):
    if not binding_error:
        return "gz.msgs10 / gz.transport13 could not be imported"
    return binding_error


# VideoWorker живе в окремому QThread для кожного відеопотоку.
# frame_ready передає QImage, номер кадру і прапорець, чи змінився кадр відносно попереднього.
class VideoWorker(QObject):
    frame_ready = Signal(QImage, int, bool)
    status_changed = Signal(str)
    error = Signal(str)

    # topic - Gazebo image topic; fps - максимальна частота оновлення GUI.
    def __init__(self, topic, fps=20):
        super().__init__()
        self.topic = topic
        self.fps = max(1, min(int(fps), 60))
        self.node = None
        self.timer = None
        self.watchdog_timer = None
        self.stop_requested = False
        self.latest_frame = None
        self.last_frame_at = None
        self.first_frame_logged = False
        self.frame_count = 0
        self.changed_frame_count = 0
        self.unchanged_frame_count = 0
        self.last_frame_hash = None
        self.latest_frame_changed = False
        self.last_diagnostics_at = 0
        self.static_warning_logged = False
        self.lock = threading.Lock()

    # Стартує Gazebo subscriber і Qt timers.
    # Slot викликається після старту video QThread.
    @Slot()
    def start(self):
        if not self.topic:
            self.status_changed.emit("Video topic is not provided")
            return

        self.stop_requested = False

        Image, Node, binding_error = load_gazebo_video_bindings()
        if Image is None or Node is None:
            self.error.emit(f"Gazebo Python transport is unavailable: {binding_error}")
            self.status_changed.emit("Gazebo Python transport is unavailable")
            return

        try:
            self.topic = normalize_gazebo_topic(self.topic)
            self.error.emit(f"Active video topic: {self.topic}")
            if "/model/mount/model/gimbal/" in self.topic:
                # Такий topic належить демонстраційному статичному gimbal mount, не рухомому Iris.
                self.error.emit(
                    "Selected video topic appears to belong to the fixed standalone "
                    "mount/gimbal model, not the moving Iris drone."
                )
            self.node = Node()
            # Кожен VideoWorker має власний Gazebo Node і власну підписку на topic.
            subscribed = self.node.subscribe(Image, self.topic, self._on_image)
            if subscribed is False:
                self.error.emit(f"Could not subscribe to Gazebo video topic: {self.topic}")
                self.status_changed.emit(f"Video unavailable: subscription failed for {self.topic}")
                return
        except Exception as exc:
            self.error.emit(f"Could not subscribe to Gazebo video topic {self.topic}: {exc}")
            self.status_changed.emit(f"Video unavailable: subscription error for {self.topic}: {exc}")
            return

        self.status_changed.emit("Waiting for video...")

        # Таймер віддає у GUI лише останній кадр з заданою частотою,
        # тому кадри не накопичуються у пам'яті.
        self.timer = QTimer(self)
        self.timer.setInterval(int(1000 / self.fps))
        self.timer.timeout.connect(self._publish_latest_frame)
        self.timer.start()

        # Watchdog періодично перевіряє, чи потік не зупинився.
        self.watchdog_timer = QTimer(self)
        self.watchdog_timer.setInterval(2000)
        self.watchdog_timer.timeout.connect(self._check_stream_health)
        self.watchdog_timer.start()

    # Позначає worker як зупинений і очищає останній кадр.
    # Викликається перед завершенням QThread.
    def request_stop(self):
        self.stop_requested = True

        with self.lock:
            self.latest_frame = None

    # Callback Gazebo Transport: викликається при кожному новому Image message.
    # msg - protobuf-повідомлення gz.msgs10.Image.
    def _on_image(self, msg):
        if self.stop_requested:
            return

        try:
            frame, frame_hash = self._message_to_qimage(msg)
        except Exception as exc:
            self.error.emit(f"Video frame decode failed: {exc}")
            return

        with self.lock:
            # Зберігаємо лише останній кадр: це запобігає росту черги при високому FPS.
            self.latest_frame = frame
            self.last_frame_at = time.monotonic()

            self.frame_count += 1
            frame_changed = False
            if self.last_frame_hash is not None:
                # Хеш допомагає відрізнити живий, але статичний потік від замороженого GUI.
                if frame_hash == self.last_frame_hash:
                    self.unchanged_frame_count += 1
                else:
                    self.changed_frame_count += 1
                    frame_changed = True
            self.last_frame_hash = frame_hash
            self.latest_frame_changed = frame_changed

        if not self.first_frame_logged:
            self.first_frame_logged = True
            # Перший кадр підтверджує, що subscriber реально отримує дані.
            self.error.emit(
                f"Video receiving frames from {self.topic}: "
                f"{int(msg.width)}x{int(msg.height)}, "
                f"format={self._pixel_format_name(msg)}, "
                f"timestamp={self._message_timestamp(msg)}"
            )

        self._log_periodic_diagnostics(msg)

    # Перетворює Gazebo Image message у QImage, придатний для показу у QLabel.
    # Повертає (QImage, frame_hash), де frame_hash використовується для діагностики змін кадру.
    def _message_to_qimage(self, msg):
        width = int(msg.width)
        height = int(msg.height)
        if width <= 0 or height <= 0:
            raise ValueError("invalid frame dimensions")

        pixel_format = self._pixel_format_name(msg)
        data = bytes(msg.data)
        # Adler32 швидкий і достатній для діагностики "кадр змінився / не змінився".
        frame_hash = zlib.adler32(data)

        if pixel_format in {"RGB_INT8", "RGB8"}:
            image_format = QImage.Format_RGB888
            bytes_per_line = msg.step or width * 3
        elif pixel_format in {"BGR_INT8", "BGR8"}:
            image_format = QImage.Format_BGR888
            bytes_per_line = msg.step or width * 3
        elif pixel_format in {"RGBA_INT8", "RGBA8"}:
            image_format = QImage.Format_RGBA8888
            bytes_per_line = msg.step or width * 4
        elif pixel_format in {"BGRA_INT8", "BGRA8"}:
            image_format = QImage.Format_BGRA8888
            bytes_per_line = msg.step or width * 4
        elif pixel_format in {"L_INT8", "L8", "MONO8"}:
            image_format = QImage.Format_Grayscale8
            bytes_per_line = msg.step or width
        else:
            raise ValueError(f"unsupported Gazebo pixel format: {pixel_format}")

        expected_size = bytes_per_line * height
        if len(data) < expected_size:
            raise ValueError("frame payload is smaller than expected")

        # .copy() потрібен, бо QImage інакше посилався б на тимчасовий bytes buffer.
        return QImage(data, width, height, bytes_per_line, image_format).copy(), frame_hash

    # Повертає текстову назву enum pixel_format_type з protobuf descriptor.
    def _pixel_format_name(self, msg):
        value = int(msg.pixel_format_type)
        descriptor = msg.DESCRIPTOR.fields_by_name.get("pixel_format_type")
        if descriptor is not None and descriptor.enum_type is not None:
            enum_value = descriptor.enum_type.values_by_number.get(value)
            if enum_value is not None:
                return enum_value.name
        return str(value)

    # Дістає timestamp з header Gazebo message, якщо він присутній.
    # Повертає рядок, бо не всі версії/топіки мають однаковий header.
    def _message_timestamp(self, msg):
        header = getattr(msg, "header", None)
        if header is None:
            return "unavailable"

        stamp = getattr(header, "stamp", None)
        if stamp is None:
            return "unavailable"

        sec = getattr(stamp, "sec", None)
        nsec = getattr(stamp, "nsec", None)
        if sec is None or nsec is None:
            return "unavailable"

        return f"{sec}.{int(nsec):09d}"

    # Раз на кілька секунд пише діагностику відео у лог панелі.
    # Це допомагає відрізнити "нема кадрів" від "кадри є, але картинка майже статична".
    def _log_periodic_diagnostics(self, msg):
        now = time.monotonic()
        if now - self.last_diagnostics_at < 5:
            return

        self.last_diagnostics_at = now
        self.error.emit(
            "Video diagnostics: "
            f"topic={self.topic}, frames={self.frame_count}, "
            f"changed={self.changed_frame_count}, unchanged={self.unchanged_frame_count}, "
            f"timestamp={self._message_timestamp(msg)}"
        )

        if (
            self.frame_count >= 30
            and self.changed_frame_count == 0
            and not self.static_warning_logged
        ):
            self.static_warning_logged = True
            # Якщо всі кадри однакові, GUI все одно живий, але джерело може бути статичним.
            self.error.emit(
                "Video stream is alive, but incoming frames appear visually identical. "
                "The camera source may be static or attached to a non-moving Gazebo model."
            )

    # Віддає останній доступний кадр у GUI з обмеженням FPS.
    # Slot викликається QTimer у video worker thread.
    @Slot()
    def _publish_latest_frame(self):
        with self.lock:
            frame = self.latest_frame
            frame_count = self.frame_count
            frame_changed = self.latest_frame_changed
            self.latest_frame = None

        if frame is not None:
            self.frame_ready.emit(frame, frame_count, frame_changed)
            self.status_changed.emit("")

    # Watchdog для випадків, коли підписка створена, але кадри не приходять або зупинились.
    @Slot()
    def _check_stream_health(self):
        with self.lock:
            last_frame_at = self.last_frame_at

        if last_frame_at is None:
            self.status_changed.emit(f"Video unavailable: no frames received from {self.topic}")
            return

        if time.monotonic() - last_frame_at > 3:
            self.status_changed.emit(f"Video stream stopped: no frames from {self.topic} for 3s")
