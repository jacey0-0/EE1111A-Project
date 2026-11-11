"""Solar MPPT demonstration dashboard for Controller.ino.
Install: python -m pip install PySide6 pyserial pyqtgraph
Run: python Dashboard_slider.py
"""
import sys
import time
from collections import deque

try:
    import serial
    from serial.tools import list_ports
    from PySide6.QtCore import Qt, QTimer
    from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
        QHBoxLayout, QGridLayout, QLabel, QPushButton, QComboBox,
        QGroupBox, QPlainTextEdit, QMessageBox, QFrame, QSlider, QScrollArea, QSizePolicy)
    import pyqtgraph as pg
except ImportError as exc:
    raise SystemExit("Missing dependency: %s\nInstall with: python -m pip install PySide6 pyserial pyqtgraph" % exc)

FIELDS = ("ms", "pvV", "pvA", "pvW", "batV", "chargeA", "loadA",
          "targetA", "buckDuty", "servo", "mode", "fault")

STYLE = """
QWidget { background: #101827; color: #e8eef9; font-family: 'Segoe UI', Arial; font-size: 12px; }
QGroupBox { border: 1px solid #2b3a51; border-radius: 10px; margin-top: 13px; padding: 12px 9px 9px; font-weight: 600; }
QGroupBox::title { subcontrol-origin: margin; left: 13px; padding: 0 5px; color: #9ab1d1; }
QPushButton { min-height: 25px; background: #243956; border: 1px solid #3a5272; border-radius: 7px; padding: 9px 12px; font-weight: 600; }
QPushButton:hover { background: #315074; }
QPushButton:disabled { color: #67768a; background: #192335; }
QPushButton#primary { background: #186a64; border-color: #299c8c; }
QPushButton#stop { background: #753640; border-color: #ac4e5c; }
QSlider::groove:horizontal { height: 9px; background: #354761; border-radius: 4px; }
QSlider::sub-page:horizontal { background: #32b7a4; border-radius: 4px; }
QSlider::handle:horizontal { background: #f4f8ff; border: 2px solid #32b7a4; width: 22px; margin: -8px 0; border-radius: 11px; }
QSlider:disabled::handle:horizontal { background: #6b7a8d; border-color: #56657a; }
QComboBox, QSpinBox { min-height: 25px; background: #1b2a40; border: 1px solid #3a5272; border-radius: 6px; padding: 6px; }
QPlainTextEdit { background: #0c1422; border: 1px solid #28374c; border-radius: 6px; font-family: Consolas, monospace; }
"""

class Dashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Solar MPPT | Live Controller Dashboard")
        self.resize(1360, 900)
        self.setMinimumSize(940, 650)
        self.ser = None
        self.buffer = bytearray()
        self.last_packet = 0.0
        self.times = deque(maxlen=300)
        self.powers = deque(maxlen=300)
        self.currents = deque(maxlen=300)
        self.loads = deque(maxlen=300)
        self.build_ui()
        self.refresh_ports()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(40)

    def build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(16, 14, 16, 14)
        outer.setSpacing(12)
        header = QHBoxLayout()
        title = QLabel("SOLAR MPPT  /  BATTERY MANAGEMENT")
        title.setStyleSheet("font-size: 21px; font-weight: 700; color: #f1f6ff;")
        header.addWidget(title)
        header.addStretch()
        self.connection = QLabel("● Disconnected")
        self.connection.setStyleSheet("color: #f2a0a0; font-weight: 700;")
        header.addWidget(self.connection)
        outer.addLayout(header)

        ports = QHBoxLayout()
        self.port = QComboBox()
        self.port.setMinimumWidth(210)
        ports.addWidget(QLabel("Serial port"))
        ports.addWidget(self.port)
        self.refresh = QPushButton("Refresh ports")
        self.refresh.clicked.connect(self.refresh_ports)
        ports.addWidget(self.refresh)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setObjectName("primary")
        self.connect_btn.clicked.connect(self.toggle_connection)
        ports.addWidget(self.connect_btn)
        ports.addWidget(QLabel("115200 baud  •  Newline"))
        ports.addStretch()
        outer.addLayout(ports)

        stats = QGridLayout()
        self.values = {}
        items = [("PV POWER", "pvW", "W"), ("PV VOLTAGE", "pvV", "V"),
                 ("PV CURRENT", "pvA", "A"), ("BATTERY VOLTAGE", "batV", "V"),
                 ("BATTERY CURRENT", "chargeA", "A"), ("LOAD CURRENT", "targetA", "A")]
        for i, (heading, key, unit) in enumerate(items):
            card = QFrame()
            card.setStyleSheet("QFrame {background: #1b2940; border: 1px solid #2b3a51; border-radius: 10px;}")
            lay = QVBoxLayout(card)
            h = QLabel(heading)
            h.setStyleSheet("border: none; color: #9fb1cb; font-size: 11px; font-weight: 700;")
            v = QLabel("— " + unit)
            v.setStyleSheet("border: none; font-size: 26px; font-weight: 700;")
            lay.addWidget(h)
            lay.addWidget(v)
            self.values[key] = (v, unit)
            stats.addWidget(card, i // 3, i % 3)
        outer.addLayout(stats)
        efficiency_row = QHBoxLayout()
        self.model_eff_label = QLabel("— %")
        self.model_eff_label.setStyleSheet("font-size: 23px; font-weight: 700; color: #f5c35c;")
        self.model_eff_note = QLabel("Efficiency")
        self.model_eff_note.setStyleSheet("color: #9fb1cb; font-size: 11px; font-weight: 700;")
        model_box = QGroupBox("MPPT")
        model_lay = QVBoxLayout(model_box)
        model_lay.addWidget(self.model_eff_note)
        model_lay.addWidget(self.model_eff_label)
        efficiency_row.addWidget(model_box, 1)
        measured_box = QGroupBox("MEASURED POWER RATIO")
        measured_lay = QVBoxLayout(measured_box)
        measured_lay.addWidget(QLabel("PV → battery (load OFF only)"))
        self.measured_ratio_label = QLabel("—")
        self.measured_ratio_label.setStyleSheet("font-size: 23px; font-weight: 700;")
        measured_lay.addWidget(self.measured_ratio_label)
        efficiency_row.addWidget(measured_box, 1)
        outer.addLayout(efficiency_row)


        middle = QHBoxLayout()
        left_panel = QWidget()
        left = QVBoxLayout(left_panel)
        left.setContentsMargins(4, 4, 10, 4)
        left.setSpacing(12)
        charger = QGroupBox("CHARGER & SAFETY")
        charger.setMinimumHeight(225)
        cg = QGridLayout(charger)
        cg.setVerticalSpacing(8)
        self.mode_label = QLabel("Mode: —")
        self.fault_label = QLabel("Fault: —")
        self.duty_label = QLabel("Buck duty: —")
        self.flow_label = QLabel("Battery: waiting for data")
        for i, label in enumerate((self.mode_label, self.fault_label, self.duty_label, self.flow_label)):
            cg.addWidget(label, i, 0, 1, 2)
        self.charge_on = self.action_button("CHARGE ON", "CHARGE ON", "primary")
        self.charge_off = self.action_button("CHARGE OFF", "CHARGE OFF", "stop")
        cg.addWidget(self.charge_on, 4, 0)
        cg.addWidget(self.charge_off, 4, 1)
        cg.addWidget(self.action_button("RESET FAULT", "RESETFAULT"), 5, 0)
        left.addWidget(charger)

        load = QGroupBox("ELECTRONIC LOAD")
        load.setMinimumHeight(195)
        lg = QGridLayout(load)
        lg.setVerticalSpacing(10)
        self.load_value_label = QLabel("0 mA")
        self.load_value_label.setStyleSheet("font-size: 20px; font-weight: 700; color: #f1f6ff;")
        self.target_label = QLabel("Controller target: —")
        lg.addWidget(QLabel("Requested current"), 0, 0)
        lg.addWidget(self.load_value_label, 0, 1, alignment=Qt.AlignmentFlag.AlignRight)
        self.load_slider = QSlider(Qt.Orientation.Horizontal)
        self.load_slider.setRange(0, 50)
        self.load_slider.setSingleStep(1)
        self.load_slider.setPageStep(5)
        self.load_slider.setTickInterval(5)
        self.load_slider.setMinimumHeight(36)
        self.load_slider.valueChanged.connect(
            lambda step: self.load_value_label.setText(f"{step * 10} mA")
        )
        lg.addWidget(self.load_slider, 1, 0, 1, 2)
        lg.addWidget(self.target_label, 2, 0, 1, 2)
        apply_btn = QPushButton("APPLY LOAD")
        apply_btn.setObjectName("primary")
        apply_btn.clicked.connect(lambda: self.send(
            "LOAD OFF" if self.load_slider.value() == 0
            else f"LOAD {self.load_slider.value() * 10}"
        ))
        lg.addWidget(apply_btn, 3, 0)
        lg.addWidget(self.action_button("LOAD OFF", "LOAD OFF", "stop"), 3, 1)
        left.addWidget(load)

        tracking = QGroupBox("SOLAR TRACKER")
        tracking.setMinimumHeight(125)
        tg = QGridLayout(tracking)
        tg.setVerticalSpacing(9)
        self.angle_label = QLabel("Current servo: —")
        tg.addWidget(self.angle_label, 0, 0, 1, 2)
        tg.addWidget(self.action_button("TRACK ON", "TRACK ON", "primary"), 1, 0)
        tg.addWidget(self.action_button("TRACK OFF", "TRACK OFF"), 1, 1)
        left.addWidget(tracking)
        left.addStretch()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setWidget(left_panel)
        scroll.setMinimumWidth(340)
        middle.addWidget(scroll, 1)

        right = QVBoxLayout()
        pg.setConfigOption("background", "#111b2a")
        pg.setConfigOption("foreground", "#aabbd0")
        self.power_plot = pg.PlotWidget(title="Solar Power (W)")
        self.power_plot.showGrid(x=True, y=True, alpha=0.2)
        self.power_plot.setMouseEnabled(x=False, y=False)
        self.power_plot.hideButtons()
        self.power_plot.setMenuEnabled(False)
        self.power_plot.setLabel("bottom", "Elapsed time", "s")
        self.power_curve = self.power_plot.plot(pen=pg.mkPen("#f5c35c", width=2))
        self.power_plot.setMinimumHeight(190)
        right.addWidget(self.power_plot)
        self.current_plot = pg.PlotWidget(title="Battery and Load Current (A)")
        self.current_plot.showGrid(x=True, y=True, alpha=0.2)
        self.current_plot.setMouseEnabled(x=False, y=False)
        self.current_plot.hideButtons()
        self.current_plot.setMenuEnabled(False)
        self.current_plot.setLabel("bottom", "Elapsed time", "s")
        self.current_plot.addLegend(offset=(-10, -10))
        self.battery_curve = self.current_plot.plot(
            pen=pg.mkPen("#4fd1a5", width=2),
            name="Battery (+ charge / − discharge)")
        self.load_curve = self.current_plot.plot(
            pen=pg.mkPen("#7caef6", width=2), name="Load target")
        self.current_plot.setMinimumHeight(190)
        right.addWidget(self.current_plot)
        middle.addLayout(right, 2)
        outer.addLayout(middle, 1)

        log_group = QGroupBox("CONTROLLER MESSAGES")
        log_layout = QVBoxLayout(log_group)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(200)
        self.log.setFixedHeight(95)
        log_layout.addWidget(self.log)
        outer.addWidget(log_group)
        self.controls = [self.charge_on, self.charge_off, apply_btn]
        self.controls += [w for w in root.findChildren(QPushButton) if w not in self.controls and w not in (self.connect_btn, self.refresh)]
        self.set_controls(False)

    def action_button(self, label, command, style=None):
        btn = QPushButton(label)
        if style:
            btn.setObjectName(style)
        btn.clicked.connect(lambda: self.send(command))
        return btn

    def set_controls(self, enabled):
        for button in self.controls:
            button.setEnabled(enabled)
        self.load_slider.setEnabled(enabled)

    def refresh_ports(self):
        current = self.port.currentData()
        self.port.clear()
        for p in sorted(list_ports.comports(), key=lambda x: x.device):
            self.port.addItem(f"{p.device} — {p.description}", p.device)
        if current:
            idx = self.port.findData(current)
            if idx >= 0:
                self.port.setCurrentIndex(idx)

    def toggle_connection(self):
        if self.ser:
            self.disconnect()
            return
        port = self.port.currentData()
        if not port:
            QMessageBox.warning(self, "No serial port", "Connect the Arduino, then click Refresh ports.")
            return
        try:
            self.ser = serial.Serial(port, 115200, timeout=0, write_timeout=0.2)
            self.buffer.clear()
            self.model_eff_label.setText("— %")
            self.measured_ratio_label.setText("—")
            self.last_packet = 0
            self.times.clear(); self.powers.clear(); self.currents.clear(); self.loads.clear()
            self.connection.setText(f"● Connected: {port} (waiting for telemetry)")
            self.connection.setStyleSheet("color: #f2c675; font-weight: 700;")
            self.connect_btn.setText("Disconnect")
            self.port.setEnabled(False)
            self.set_controls(True)
            self.message("Connected. Arduino may reset when the serial port opens.")
        except serial.SerialException as exc:
            self.ser = None
            QMessageBox.critical(self, "Connection failed", str(exc))

    def disconnect(self):
        if self.ser:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None
        self.buffer.clear()
        self.connection.setText("● Disconnected")
        self.connection.setStyleSheet("color: #f2a0a0; font-weight: 700;")
        self.connect_btn.setText("Connect")
        self.port.setEnabled(True)
        self.set_controls(False)
        self.model_eff_label.setText("— %")
        self.measured_ratio_label.setText("—")

    def send(self, cmd):
        if not self.ser:
            return
        try:
            self.ser.write((cmd + "\n").encode("ascii"))
            self.message("→ " + cmd)
        except (serial.SerialException, serial.SerialTimeoutException, OSError) as exc:
            self.message("Serial write failed: " + str(exc))
            self.disconnect()

    def message(self, text):
        self.log.appendPlainText(text)

    def poll(self):
        if not self.ser:
            return
        try:
            incoming = self.ser.read(min(self.ser.in_waiting, 4096))
            if incoming:
                self.buffer.extend(incoming)
            if len(self.buffer) > 16384:
                self.buffer.clear()
                self.message("Discarded oversized serial buffer")
            processed = 0
            while b"\n" in self.buffer and processed < 50:
                raw, _, rest = self.buffer.partition(b"\n")
                self.buffer = bytearray(rest)
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    self.handle_line(line)
                processed += 1
            if self.last_packet and time.monotonic() - self.last_packet > 3:
                self.connection.setText("● Connected, telemetry stale")
                self.connection.setStyleSheet("color: #f2c675; font-weight: 700;")
        except (serial.SerialException, OSError) as exc:
            self.message("Serial connection lost: " + str(exc))
            self.disconnect()

    def handle_line(self, line):
        if line.startswith("#"):
            self.message(line)
            return
        if line.startswith("ms,"):
            return
        parts = line.split(",")
        if len(parts) != len(FIELDS):
            self.message("Unrecognized serial line: " + line[:120])
            return
        try:
            data = {k: float(v) for k, v in zip(FIELDS[:9], parts[:9])}
            data["servo"] = int(parts[9])
            data["mode"] = parts[10]
            data["fault"] = int(parts[11])
        except ValueError:
            self.message("Invalid telemetry: " + line[:120])
            return
        self.last_packet = time.monotonic()
        self.connection.setText("● Live telemetry")
        self.connection.setStyleSheet("color: #4fd1a5; font-weight: 700;")
        for key, (label, unit) in self.values.items():
            label.setText(f"{data[key]:.3f} {unit}")
        import math
        pv_power = data["pvW"]
        bat_power = data["batV"] * data["chargeA"]
        valid_ratio = (math.isfinite(pv_power) and math.isfinite(bat_power)
                       and pv_power > 0.02 and data["targetA"] <= 0.001
                       and bat_power > 0)
        if valid_ratio:
            ratio = 100.0 * bat_power / pv_power
            self.model_eff_label.setText(f"{ratio:.1f}%")
        else:
            self.model_eff_label.setText("— %")
        pv_power = data["pvW"]
        bat_power = data["batV"] * data["chargeA"]
        if (math.isfinite(pv_power) and math.isfinite(bat_power)
                and pv_power > 0.02 and data["targetA"] <= 0.001
                and bat_power > 0):
            self.measured_ratio_label.setText(f"{100 * bat_power / pv_power:.1f}%")
        else:
            self.measured_ratio_label.setText("—")
        self.mode_label.setText("Mode: " + data["mode"])
        self.fault_label.setText("Fault: " + ("LATCHED — inspect system" if data["fault"] else "None"))
        self.fault_label.setStyleSheet("color: #f49c9c;" if data["fault"] else "color: #4fd1a5;")
        self.duty_label.setText(f"Buck duty command: {int(data['buckDuty'])} / 160")
        current = data["chargeA"]
        flow = "Charging" if current > 0.01 else "Discharging" if current < -0.01 else "Near zero"
        self.flow_label.setText("Battery: " + flow)
        self.target_label.setText(f"Controller target: {data['targetA'] * 1000:.0f} mA")
        self.angle_label.setText(f"Current servo: {data['servo']}°")
        self.times.append(data["ms"] / 1000.0)
        self.powers.append(data["pvW"])
        self.currents.append(current)
        self.loads.append(data["targetA"])
        self.power_curve.setData(list(self.times), list(self.powers))
        self.battery_curve.setData(list(self.times), list(self.currents))
        self.load_curve.setData(list(self.times), list(self.loads))
        self.power_plot.getPlotItem().autoRange(padding=0.08)
        self.current_plot.getPlotItem().autoRange(padding=0.08)

    def closeEvent(self, event):
        self.disconnect()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    window = Dashboard()
    window.show()
    sys.exit(app.exec())
