import sys
from PyQt6.QtWidgets import (
    QApplication, 
    QWidget, 
    QHBoxLayout,
    QLabel
)
from PyQt6.QtGui import QPainter, QPen, QColor
from PyQt6.QtCore import Qt, QRect, QSize, QRectF

class BatteryWidget(QWidget):
    def __init__(self, parent = None):
        super().__init__(parent)
        self.percentage = 75
        self.voltage = 12

        self.setMinimumSize(200, 60)

    def setVoltage(self, value):
        self.voltage = max(0, min(100, value))
        self.update()

    def setPercentage(self, value):
        self.percentage = value
        self.update()
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        width = self.width()
        height = self.height()

        text_width = 70
        tip_width = 14
        battery_width = width - text_width - tip_width
        battery_height = height * 0.65
        battery_x = text_width
        battery_y = (height - battery_height) / 2

        #-------------------
        #VOLTAGE AND PERCENT

        #set text black
        painter.setPen(Qt.GlobalColor.black)

        #text input
        text = f"{self.percentage}% \n {self.voltage: 0.2f}V"
        
        #make section for text
        painter.drawText(
            5, 0, text_width - 10, height, Qt.AlignmentFlag.AlignVCenter, text
        )

        #---------------
        #BATTERY OUTLINE

        pen = QPen(Qt.GlobalColor.black, 2)
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        #outer battery shell
        body_rectangle = QRectF(battery_x, battery_y, battery_width, battery_height)
        #rounded corners
        painter.drawRoundedRect(body_rectangle, 4, 4)

        #Battery Tip
        tip_rectangle = QRectF(
            battery_x + battery_width, 
            battery_y + battery_height * 0.25, 
            tip_width, 
            battery_height * 0.5
            )
        
        painter.drawRect(tip_rectangle)

        #----------------
        #Battery Segments

        segment_count = 5
        spacing = 3
        usable_width = battery_width - spacing * (segment_count + 1)
        segment_width = usable_width / segment_count
        segment_height = battery_height - 2 * spacing

        #calculate bars to percentage
        filled_segments = int((self.percentage / 100) * segment_count)

        #loop for 5 segments
        for i in range(segment_count):

            # X position of each segment
            x_pos = battery_x + spacing + (segment_width + spacing) * i
            # Y position (vertical centering inside the battery)
            y_pos = battery_y + spacing

            # Rectangle for this segment
            segment_rectangle = QRectF(x_pos, y_pos, segment_width, segment_height)

            # If this segment is "filled", draw green; otherwise draw gray
            if i < filled_segments:
                #Filled = green
                painter.setBrush(QColor(70, 200, 70))
            else:
                #Empty = light gray
                painter.setBrush(QColor(200, 200, 200))

            # Draw the segment rectangle
            painter.drawRect(segment_rectangle)     

#---------------------
#DEMO APPLICATION TEST
if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = QWidget()
    layout = QHBoxLayout(window)

    # Create battery widget instance using example values
    battery = BatteryWidget()
    battery.setPercentage(80)
    battery.setVoltage(12)
    battery.show()

    # Add widget to window layout
    layout.addWidget(battery)

    window.resize(350, 120)
    window.setWindowTitle("Battery Widget (PyQt6)")
    window.show()

    sys.exit(app.exec())