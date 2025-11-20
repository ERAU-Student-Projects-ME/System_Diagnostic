'''
Control Node
Path Planer Node
YOLO Node
GBCACHE
Mapper
ROIS(Fusion)

What we need for each node:
Label
Widget type LED or light up text bar
Status color

'''
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QHBoxLayout,
    QVBoxLayout, QFrame
)
from PyQt6.QtGui import QColor, QPalette
from PyQt6.QtCore import Qt
import sys


class LEDIndicator(QFrame):
    """Simple circular LED indicator"""
    def __init__(self, size=20, color="gray"):
        super().__init__()
        self.size = size
        self.setFixedSize(size, size)
        self.setStyleSheet(f"""
            background-color: {color};
            border-radius: {size // 2}px;
            border: 1px solid black;
        """)

    def set_color(self, color: str):
        self.setStyleSheet(f"""
            background-color: {color};
            border-radius: {self.size // 2}px;
            border: 1px solid black;
        """)


class NodeStatusWidget(QWidget):
    """A widget representing a single node with label + LED"""
    def __init__(self, node_name: str):
        super().__init__()

        self.label = QLabel(node_name)
        self.led = LEDIndicator()

        layout = QHBoxLayout()
        layout.addWidget(self.label)
        layout.addStretch(1)
        layout.addWidget(self.led)

        self.setLayout(layout)

    def set_status(self, status: str):
        """Set status color by keyword."""
        colors = {
            "ok": "green",
            "warning": "yellow",
            "error": "red",
            "offline": "gray"
        }
        self.led.set_color(colors.get(status.lower(), "gray"))


class NodePanel(QWidget):
    """Main widget containing all nodes"""
    def __init__(self):
        super().__init__()

        self.nodes = {
            "Control Node": NodeStatusWidget("Control Node"),
            "Path Planner Node": NodeStatusWidget("Path Planner Node"),
            "YOLO Node": NodeStatusWidget("YOLO Node"),
            "GBCACHE": NodeStatusWidget("GBCACHE"),
            "Mapper": NodeStatusWidget("Mapper"),
            "ROIS (Fusion)": NodeStatusWidget("ROIS (Fusion)")
        }

        layout = QVBoxLayout()
        for node in self.nodes.values():
            layout.addWidget(node)

        layout.addStretch(1)
        self.setLayout(layout)

        # Example initial statuses
        self.nodes["Control Node"].set_status("ok")
        self.nodes["Path Planner Node"].set_status("warning")
        self.nodes["YOLO Node"].set_status("error")
        self.nodes["GBCACHE"].set_status("offline")
        self.nodes["Mapper"].set_status("ok")
        self.nodes["ROIS (Fusion)"].set_status("ok")


if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = NodePanel()
    window.setWindowTitle("Node Status Panel")
    window.resize(300, 250)
    window.show()

    sys.exit(app.exec())

