#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QGridLayout, QProgressBar, QVBoxLayout
)
from PyQt5.QtCore import QTimer, Qt
import math
from collections import deque
import pyqtgraph as pg
from minion_diagnostic_msg.msg import SystemStatus  # Replace with your package


# -------------------------------
# GUI Class
# -------------------------------
class SystemStatusGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("📊 System Monitor Dashboard")
        self.resize(700, 600)

        main_layout = QVBoxLayout()
        grid = QGridLayout()

        # Progress bars and labels
        self.cpu_bar = QProgressBar()
        self.gpu_bar = QProgressBar()
        self.ram_bar = QProgressBar()
        self.disk_bar = QProgressBar()
        self.battery_bar = QProgressBar()
        self.temp_label = QLabel("---")
        self.charge_label = QLabel("---")
        self.linux_label = QLabel("---")
        self.time_label = QLabel("---")

        # Align labels
        for lbl in [self.temp_label, self.charge_label, self.linux_label, self.time_label]:
            lbl.setAlignment(Qt.AlignCenter)

        # Add widgets to grid
        grid.addWidget(QLabel("CPU Usage (%)"), 0, 0); grid.addWidget(self.cpu_bar, 0, 1)
        grid.addWidget(QLabel("GPU Usage (%)"), 1, 0); grid.addWidget(self.gpu_bar, 1, 1)
        grid.addWidget(QLabel("Temperature (°C)"), 2, 0); grid.addWidget(self.temp_label, 2, 1)
        grid.addWidget(QLabel("Battery (%)"), 3, 0); grid.addWidget(self.battery_bar, 3, 1)
        grid.addWidget(QLabel("Charging"), 4, 0); grid.addWidget(self.charge_label, 4, 1)
        grid.addWidget(QLabel("Linux Version"), 5, 0); grid.addWidget(self.linux_label, 5, 1)
        grid.addWidget(QLabel("Current Time"), 6, 0); grid.addWidget(self.time_label, 6, 1)
        grid.addWidget(QLabel("RAM Usage (%)"), 7, 0); grid.addWidget(self.ram_bar, 7, 1)
        grid.addWidget(QLabel("Storage Used (%)"), 8, 0); grid.addWidget(self.disk_bar, 8, 1)

        main_layout.addLayout(grid)

        # === Live plots ===
        self.plot_widget = pg.GraphicsLayoutWidget()
        main_layout.addWidget(self.plot_widget)

        # CPU plot
        self.cpu_plot = self.plot_widget.addPlot(title="CPU Usage (%)")
        self.cpu_curve = self.cpu_plot.plot(pen='y')
        self.cpu_history = deque([0]*100, maxlen=100)

        self.plot_widget.nextRow()

        # GPU plot
        self.gpu_plot = self.plot_widget.addPlot(title="GPU Usage (%)")
        self.gpu_curve = self.gpu_plot.plot(pen='c')
        self.gpu_history = deque([0]*100, maxlen=100)

        self.plot_widget.nextRow()

        # RAM plot
        self.ram_plot = self.plot_widget.addPlot(title="RAM Usage (%)")
        self.ram_curve = self.ram_plot.plot(pen='m')
        self.ram_history = deque([0]*100, maxlen=100)

        self.setLayout(main_layout)

    def update_from_msg(self, msg: SystemStatus):
        # Helper for progress bars
        def set_bar(bar, value):
            if value != value:  # NaN
                bar.setFormat("N/A")
                bar.setValue(0)
            else:
                bar.setFormat(f"{value:.1f}%")
                bar.setValue(int(value))

        set_bar(self.cpu_bar, msg.cpu_usage)
        set_bar(self.gpu_bar, msg.gpu_usage)
        set_bar(self.ram_bar, msg.ram_usage)
        set_bar(self.disk_bar, msg.storage_used)
        set_bar(self.battery_bar, msg.battery_percentage)

        # Temperature color coding
        temp = msg.temperature
        if temp != temp:  # NaN
            self.temp_label.setText("N/A")
            self.temp_label.setStyleSheet("")
        else:
            self.temp_label.setText(f"{temp:.1f} °C")
            if temp > 80:
                self.temp_label.setStyleSheet("color: red; font-weight: bold;")
            elif temp > 60:
                self.temp_label.setStyleSheet("color: orange; font-weight: bold;")
            else:
                self.temp_label.setStyleSheet("color: green;")

        # Battery charging
        self.charge_label.setText("Yes" if msg.battery_plugged else "No")
        self.charge_label.setStyleSheet("color: blue;" if msg.battery_plugged else "color: gray;")

        # OS + Time
        self.linux_label.setText(msg.linux_version)
        self.time_label.setText(msg.current_time)

        # Update histories and plots
        self.cpu_history.append(msg.cpu_usage if not math.isnan(msg.cpu_usage) else 0)
        self.cpu_curve.setData(list(self.cpu_history))

        self.gpu_history.append(msg.gpu_usage if not math.isnan(msg.gpu_usage) else 0)
        self.gpu_curve.setData(list(self.gpu_history))

        self.ram_history.append(msg.ram_usage if not math.isnan(msg.ram_usage) else 0)
        self.ram_curve.setData(list(self.ram_history))


# -------------------------------
# ROS Node Class
# -------------------------------
class SystemStatusNode(Node):
    def __init__(self, gui: SystemStatusGUI):
        super().__init__("system_status_gui_node")
        self.gui = gui
        self.sub = self.create_subscription(
            SystemStatus, "system_status", self.listener_callback, 10
        )

    def listener_callback(self, msg: SystemStatus):
        self.gui.update_from_msg(msg)


# -------------------------------
# Main
# -------------------------------
def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    gui = SystemStatusGUI()
    gui.show()

    node = SystemStatusNode(gui)

    # Timer to spin ROS inside Qt loop
    qtimer = QTimer()
    qtimer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    qtimer.start(50)  # 20 Hz

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
