import sys
from PyQt6.QtWidgets import (
    QApplication, 
    QWidget, 
    QHBoxLayout,
    QSlider, 
    QLabel
)
from PyQt5.QtGui import QPainter, QPen, QColor, QBrush
from PyQt6.QtCore import Qt, QRect, QSize

class BatteryWidget(QWidget):
    def __init__(self):
        super().__init__(self)

    def sizeHint(self):
        return QSize(40,120)
    
    def paintEvent(self, e):
        painter = QPainter(self)
        brush = QBrush()
        brush.setColor(QColor('black'))
        brush.setStyle(Qt.BrushStyle.SolidPattern)
        rect = QRect(0, 0, painter.device().width(), painter.device().height())
        painter.fillRect(rect, brush)
    
class BatteryWidgetDemo(QWidget, BatteryWidget):
    def __init__(self):
        super().__init__(self)

        app = QApplication()
        battery = BatteryWidget()
        battery.show()
        app.exec()