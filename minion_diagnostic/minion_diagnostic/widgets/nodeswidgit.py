'''
Nodes Widgit: Control Node, Path Planner Node, YOLO Node, GBCACHE, Mapper, ROIS(Fusion)
For Each Node -> Label, Widget Type, Status Color [Binary; Is it delivering data? Yes or No]
'''

import sys
from PyQt6.QtCore import Qt, pyqtSignal, QObject
from PyQt6.QtGui import QColor, QPalette
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QHBoxLayout,
    QVBoxLayout, QFrame, QGridLayout
)

class NodeStatusWidget(QWidget):
    # Node Widget
    def __init__(self, node_name: str):
        super().__init__()

        self.label = QLabel(node_name)

        # Small Padding + Bold Text
        self.label.setContentsMargins(5, 3, 5, 3)
        self.label.setStyleSheet("""
            border-radius: 4px;
            border: 1px solid black;
            background-color: gray;
            font-weight: bold;
        """)

        # Small Margins 
        layout = QHBoxLayout()
        layout.setContentsMargins(2, 2, 2, 2)
        layout.addWidget(self.label)
        self.setLayout(layout)

        # Background Color Options: Green(Yes,1) or Red(No,2)
        self.status_map = {
            1: "green",
            2: "red",
        }

    def set_status_from_code(self, code: int):
        # Apply Background Color Using Status Code
        color = self.status_map.get(code, "gray")
        self.label.setStyleSheet(f"""
            border-radius: 4px;
            border: 1px solid black;
            background-color: {color};
            font-weight: bold;
        """)


class NodeStatusBridge(QObject):
    status_update = pyqtSignal(str, int)


# Main Node Widget; all nodes in a 3x2 grid
class NodePanel(QWidget):
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

        # 3x2 Grid Layout
        layout = QGridLayout()
        layout.setSpacing(8)

        rows = 2
        cols = 3
        index = 0

        for r in range(rows):
            for c in range(cols):
                name = self.node_names[index]
                layout.addWidget(self.nodes[name], r, c)
                index += 1

        self.setLayout(layout)

        # Signal Bridge
        self.bridge = NodeStatusBridge()
        self.bridge.status_update.connect(self.update_node_from_signal)

        # Demo Statuses
        self.nodes["Control Node"].set_status_from_code(1)
        self.nodes["Path Planner Node"].set_status_from_code(2)
        self.nodes["YOLO Node"].set_status_from_code(1)
        self.nodes["GBCACHE"].set_status_from_code(2)
        self.nodes["Mapper"].set_status_from_code(1)
        self.nodes["ROIS (Fusion)"].set_status_from_code(1)

    def update_node_from_signal(self, node_name, status_code):
        if node_name in self.nodes:
            self.nodes[node_name].set_status_from_code(status_code)

# Change the status of a node by doing self.node["YOLO Node"].set_status_from_code(#(1-2))
# 1 is green working
# 2 is red not working

if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = NodePanel()
    window.setWindowTitle("Node Status Panel")
    window.resize(450, 200)
    window.show()

    sys.exit(app.exec())
