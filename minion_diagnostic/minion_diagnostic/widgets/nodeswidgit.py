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


from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QHBoxLayout,
    QVBoxLayout, QFrame, QGridLayout
)
from PyQt6.QtCore import Qt, pyqtSignal, QObject
import sys


class NodeStatusWidget(QWidget):
    """Node widget where the label background changes color."""
    def __init__(self, node_name: str):
        super().__init__()

        self.label = QLabel(node_name)

        # Smaller padding + bold text
        self.label.setContentsMargins(5, 3, 5, 3)
        self.label.setStyleSheet("""
            border-radius: 4px;
            border: 1px solid black;
            background-color: gray;
            font-weight: bold;
        """)

        layout = QHBoxLayout()
        layout.setContentsMargins(2, 2, 2, 2)  # smaller outer margins
        layout.addWidget(self.label)
        self.setLayout(layout)

        # Integer → color mapping
        self.status_map = {
            1: "green",   # ok
            2: "yellow",  # warning
            3: "red",     # error
            4: "gray"     # offline
        }

    def set_status_from_code(self, code: int):
        """Apply background color based on status code."""
        color = self.status_map.get(code, "gray")
        self.label.setStyleSheet(f"""
            border-radius: 4px;
            border: 1px solid black;
            background-color: {color};
            font-weight: bold;
        """)


class NodeStatusBridge(QObject):
    status_update = pyqtSignal(str, int)


class NodePanel(QWidget):
    """Main widget containing all nodes in a 3x2 grid."""
    def __init__(self):
        super().__init__()

        self.node_names = [
            "Control Node",
            "Path Planner Node",
            "YOLO Node",
            "GBCACHE",
            "Mapper",
            "ROIS (Fusion)"
        ]

        self.nodes = {name: NodeStatusWidget(name) for name in self.node_names}

        # ---------- 3x2 Grid Layout ----------
        layout = QGridLayout()
        layout.setSpacing(8)  # tighter spacing between boxes

        rows = 2
        cols = 3
        index = 0

        for r in range(rows):
            for c in range(cols):
                name = self.node_names[index]
                layout.addWidget(self.nodes[name], r, c)
                index += 1

        self.setLayout(layout)

        # Signal bridge
        self.bridge = NodeStatusBridge()
        self.bridge.status_update.connect(self.update_node_from_signal)

        # Demo statuses
        self.nodes["Control Node"].set_status_from_code(1)
        self.nodes["Path Planner Node"].set_status_from_code(2)
        self.nodes["YOLO Node"].set_status_from_code(3)
        self.nodes["GBCACHE"].set_status_from_code(4)
        self.nodes["Mapper"].set_status_from_code(1)
        self.nodes["ROIS (Fusion)"].set_status_from_code(1)

    def update_node_from_signal(self, node_name, status_code):
        if node_name in self.nodes:
            self.nodes[node_name].set_status_from_code(status_code)

# Change the status of a node by doing self.node["YOLO Node"].set_status_from_code(#(1-4))
# 1 is green working
# 2 is yellow warning
# 3 is red error
# 4 is gray offline

if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = NodePanel()
    window.setWindowTitle("Node Status Panel")
    window.resize(450, 200)
    window.show()

    sys.exit(app.exec())

