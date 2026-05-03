# Віджет однієї панелі дрона у GUI.
# Він показує телеметрію, лог, карту маршруту/поточної позиції та відео з Gazebo.
from datetime import datetime
import math

import pyqtgraph as pg
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from drone_client import gps_to_enu


# DronePanel не працює з MAVLink напряму.
# Він лише показує дані і надсилає сигнали connect_requested/start_requested,
# які головне вікно підключає до відповідного DroneWorker.
class DronePanel(QWidget):
    connect_requested = Signal()
    start_requested = Signal()

    # title - назва панелі ("DRONE 1"/"DRONE 2"); route - маршрут для побудови карти.
    def __init__(self, title, route, parent=None):
        super().__init__(parent)
        self.route = route
        self.ref_lat = route["points"][0]["lat"]
        self.ref_lon = route["points"][0]["lon"]
        self.current_marker = None
        self.video_pixmap = None
        self.video_overlay = ""

        self.title_label = QLabel(title)
        self.status_label = QLabel("Disconnected")

        self.connect_button = QPushButton("Connect")
        self.start_button = QPushButton("Start Mission")
        self.start_button.setEnabled(False)

        self.telemetry_values = {}
        # Верхня частина панелі: телеметрія, кнопки та текстовий лог.
        telemetry_box = self._build_telemetry_box()
        log_box = self._build_log_box()
        plot_box = self._build_plot_box()
        video_box = self._build_video_box()

        button_row = QHBoxLayout()
        button_row.addWidget(self.connect_button)
        button_row.addWidget(self.start_button)

        control_column = QVBoxLayout()
        control_column.addLayout(button_row)
        control_column.addWidget(log_box)

        header = QHBoxLayout()
        header.addWidget(self.title_label)
        header.addStretch()
        header.addWidget(self.status_label)

        top_grid = QGridLayout()
        top_grid.addWidget(telemetry_box, 0, 0)
        top_grid.addLayout(control_column, 0, 1)
        top_grid.setColumnStretch(1, 1)

        layout = QVBoxLayout(self)
        layout.addLayout(header)
        layout.addLayout(top_grid)

        media_row = QHBoxLayout()
        # Карта і відео розміщені поруч, щоб кожен дрон мав власний повний блок спостереження.
        media_row.addWidget(plot_box, 1)
        media_row.addWidget(video_box, 1)
        layout.addLayout(media_row)

        # Кнопки відправляють Qt-сигнали, а не викликають worker напряму.
        self.connect_button.clicked.connect(self.connect_requested.emit)
        self.start_button.clicked.connect(self.start_requested.emit)

        self._plot_route()

    # Створює блок поточної телеметрії і зберігає QLabel для кожного поля.
    # Повертає QGroupBox, який вставляється у layout панелі.
    def _build_telemetry_box(self):
        box = QGroupBox("Current Telemetry")
        form = QFormLayout(box)

        fields = [
            ("mode", "Mode"),
            ("armed", "Armed"),
            ("lat", "Latitude"),
            ("lon", "Longitude"),
            ("alt", "Altitude (m)"),
            ("roll", "Roll"),
            ("pitch", "Pitch"),
            ("yaw", "Yaw"),
            ("speed", "V (m/s)"),
            ("vx", "Vx (m/s)"),
            ("vy", "Vy (m/s)"),
            ("vz", "Vz (m/s)"),
            ("battery", "Battery (%)"),
            ("imu_acc", "IMU Acc"),
            ("imu_gyro", "IMU Gyro"),
        ]

        for key, label in fields:
            value = QLabel("-")
            # QLabel зберігаються у словнику, щоб update_telemetry швидко оновлював значення.
            self.telemetry_values[key] = value
            form.addRow(label + ":", value)

        return box

    # Створює текстовий лог для повідомлень MAVLink, місії та відео.
    def _build_log_box(self):
        box = QGroupBox("Telemetry Log")
        layout = QVBoxLayout(box)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        layout.addWidget(self.log_view)
        return box

    # Створює pyqtgraph-карту траєкторії у локальних координатах East/North.
    def _build_plot_box(self):
        box = QGroupBox("Trajectory Map")
        layout = QVBoxLayout(box)
        self.plot = pg.PlotWidget()
        self.plot.setMinimumSize(360, 280)
        self.plot.setLabel("bottom", "East", units="m")
        self.plot.setLabel("left", "North", units="m")
        self.plot.showGrid(x=True, y=True)
        self.plot.getPlotItem().getViewBox().setAspectLocked(True, ratio=1)
        layout.addWidget(self.plot)
        return box

    # Створює область відео. Якщо кадри ще не прийшли, показується текстовий статус.
    def _build_video_box(self):
        box = QGroupBox("Video Feed")
        layout = QVBoxLayout(box)
        self.video_label = QLabel("No Video")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setWordWrap(True)
        self.video_label.setMinimumSize(360, 280)
        self.video_label.setStyleSheet(
            "QLabel { background: #111; color: #ccc; border: 1px solid #333; }"
        )
        layout.addWidget(self.video_label)
        return box

    # Малює запланований маршрут на карті і створює маркер поточної позиції.
    def _plot_route(self):
        east_values = []
        north_values = []
        points = self.route["points"]

        for point in points:
            # GPS точки маршруту переводяться у метри відносно першої точки.
            east, north = gps_to_enu(
                point["lat"],
                point["lon"],
                self.ref_lat,
                self.ref_lon,
            )
            east_values.append(east)
            north_values.append(north)

        self.plot.plot(
            east_values,
            north_values,
            pen=pg.mkPen("#2d7ff9", width=2),
            symbol="o",
            symbolBrush="#2d7ff9",
            name="Planned Path",
        )

        for index, point in enumerate(points):
            # Якщо маршрут замкнений і остання точка дублює першу, не дублюємо підпис.
            if index == len(points) - 1 and self._is_duplicate_of_first(point):
                continue

            east, north = gps_to_enu(
                point["lat"],
                point["lon"],
                self.ref_lat,
                self.ref_lon,
            )
            label = pg.TextItem(str(index + 1), anchor=(0.5, -0.6), color="#dddddd")
            label.setPos(east, north)
            self.plot.addItem(label)

        # Маркер поточної позиції оновлюється методом update_position().
        self.current_marker = self.plot.plot(
            [],
            [],
            pen=None,
            symbol="o",
            symbolSize=14,
            symbolBrush="#24c743",
            name="Current Position",
        )

    # Перевіряє, чи точка збігається з першою точкою маршруту.
    # Використовується лише для акуратного підпису замкнених маршрутів.
    def _is_duplicate_of_first(self, point):
        first = self.route["points"][0]
        return point["lat"] == first["lat"] and point["lon"] == first["lon"]

    # Викликається після успішного MAVLink-підключення.
    def set_connected(self):
        self.status_label.setText("Connected")
        self.connect_button.setEnabled(False)
        self.start_button.setEnabled(True)

    # Вимикає кнопку старту, поки місія виконується.
    def set_mission_running(self):
        self.start_button.setEnabled(False)

    # Дозволяє повторний старт після завершення/зупинки місії.
    def set_mission_finished(self):
        self.start_button.setEnabled(True)

    # Оновлює короткий статус у заголовку панелі.
    def set_status(self, status):
        self.status_label.setText(status)

    # Додає рядок у лог панелі з локальним часом.
    # message - текст від worker або відеопотоку.
    def add_log(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        if message:
            self.log_view.append(f"{timestamp}  {message}")
        else:
            self.log_view.append("")

        scrollbar = self.log_view.verticalScrollBar()
        # Автопрокрутка тримає останні повідомлення видимими.
        scrollbar.setValue(scrollbar.maximum())

    # Оновлює всі текстові поля телеметрії з snapshot, який надсилає DroneWorker.
    def update_telemetry(self, telemetry):
        self.telemetry_values["mode"].setText(str(telemetry.get("mode") or "-"))
        armed = telemetry.get("armed")
        self.telemetry_values["armed"].setText("-" if armed is None else str(armed))

        battery = telemetry.get("battery")
        if battery:
            self.telemetry_values["battery"].setText(str(battery.get("remaining_percent", "-")))

        attitude = telemetry.get("attitude")
        if attitude:
            self.telemetry_values["roll"].setText(f"{attitude['roll']:.3f}")
            self.telemetry_values["pitch"].setText(f"{attitude['pitch']:.3f}")
            self.telemetry_values["yaw"].setText(f"{attitude['yaw']:.3f}")

        position = telemetry.get("position")
        if position:
            # Швидкість показується як модуль вектора vx/vy/vz.
            speed = math.sqrt(position["vx"] ** 2 + position["vy"] ** 2 + position["vz"] ** 2)
            self.telemetry_values["lat"].setText(f"{position['lat']:.7f}")
            self.telemetry_values["lon"].setText(f"{position['lon']:.7f}")
            self.telemetry_values["alt"].setText(f"{position['relative_alt_m']:.2f}")
            self.telemetry_values["speed"].setText(f"{speed:.2f}")
            self.telemetry_values["vx"].setText(f"{position['vx']:.2f}")
            self.telemetry_values["vy"].setText(f"{position['vy']:.2f}")
            self.telemetry_values["vz"].setText(f"{position['vz']:.2f}")

        imu = telemetry.get("imu")
        if imu:
            self.telemetry_values["imu_acc"].setText(
                f"{imu['xacc']}, {imu['yacc']}, {imu['zacc']}"
            )
            self.telemetry_values["imu_gyro"].setText(
                f"{imu['xgyro']}, {imu['ygyro']}, {imu['zgyro']}"
            )

    # Оновлює зелений маркер на карті.
    # east/north - локальні координати у метрах відносно стартової точки маршруту.
    def update_position(self, east, north):
        self.current_marker.setData([east], [north])

    # Отримує новий кадр відео від VideoWorker і зберігає його як QPixmap.
    # frame_count/frame_changed використовуються тільки для діагностичного overlay.
    def set_video_frame(self, image, frame_count=None, frame_changed=None):
        self.video_pixmap = QPixmap.fromImage(image)
        if frame_count is not None:
            state = "changed" if frame_changed else "same"
            self.video_overlay = f"frames: {frame_count} | {state}"
        self._refresh_video_pixmap()

    # Показує текстовий стан відео, наприклад "No Video" або повідомлення про помилку.
    def set_video_status(self, status):
        if not status:
            return
        self.video_pixmap = None
        self.video_label.clear()
        self.video_label.setText(status)

    # Масштабує останній кадр під розмір QLabel і малює діагностичний overlay.
    def _refresh_video_pixmap(self):
        if self.video_pixmap is None:
            return

        scaled = self.video_pixmap.scaled(
            self.video_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        if self.video_overlay:
            # Overlay візуально підтверджує, що GUI отримує нові кадри.
            painter = QPainter(scaled)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(8, 8, 150, 24, QColor(0, 0, 0, 160))
            painter.setPen(QPen(QColor("#ffffff")))
            painter.drawText(16, 25, self.video_overlay)
            painter.end()
        self.video_label.setPixmap(scaled)

    # При зміні розміру панелі повторно масштабуємо останній відеокадр.
    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._refresh_video_pixmap()
