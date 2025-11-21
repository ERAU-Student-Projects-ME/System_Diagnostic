from PyQt5.QtWidgets import QProgressBar
from PyQt5.QtCore import QPropertyAnimation, pyqtProperty


def _interp(a, b, t):
    """Interpolates between two values."""
    return a + (b - a) * t


class SmoothProgressBar(QProgressBar):
    def __init__(self, parent=None, color="dodgerblue", choropleth=False, temp_range=(30.0, 80.0)):
        """Simplified progress bar.

        If `choropleth` is True the chunk color will be a gradient from green->red
        based on percent (0..100).
        """
        super().__init__(parent)
        self._value = 0.0
        self.animation = QPropertyAnimation(self, b"value", self)
        self.animation.setDuration(150)  # ms
        self.setTextVisible(True)

        self.setMinimum(0)
        self.setMaximum(100)

        self._solid_color = color
        self._choropleth = bool(choropleth)
        self._min_temp, self._max_temp = temp_range

        # Start with a default style
        self._apply_stylesheet(self._solid_color)

    def _apply_stylesheet(self, chunk_color_hex):
        """Applies stylesheet with a white text color."""
        try:
            self._last_chunk_color = chunk_color_hex
        except Exception:
            self._last_chunk_color = chunk_color_hex
        self.setStyleSheet(f"""
            QProgressBar {{
                border: 1px solid #444;
                border-radius: 5px;
                background-color: #2b2b2b;
                text-align: center;
                color: #ffffff;
            }}
            QProgressBar::chunk {{
                background-color: {chunk_color_hex};
                border-radius: 5px;
                margin: 0px;
            }}
        """)

    def setTargetValue(self, value):
        """Animate to the new value."""
        self.animation.stop()
        self.animation.setStartValue(self._value)
        self.animation.setEndValue(value)
        self.animation.start()

    def getValue(self):
        return self._value

    def setValue(self, value):
        self._value = float(value)
        super().setValue(int(round(value)))

    value = pyqtProperty(float, getValue, setValue)

    def set_temperature(self, temp, min_temp=None, max_temp=None):
        """Set the visual color of the bar according to temperature.

        temp: current temperature (float)
        min_temp: temperature at which bar is fully green (<= becomes green)
        max_temp: temperature at which bar is fully red (>= becomes red).
        Returns the chunk color hex string.
        """
        if min_temp is None:
            min_temp = self._min_temp
        if max_temp is None:
            max_temp = self._max_temp

        # clamp t 0..1
        if max_temp == min_temp:
            t = 1.0
        else:
            t = (temp - min_temp) / (max_temp - min_temp)
        t = max(0.0, min(1.0, t))

        # interpolate from green (0,200,120) to red (220,40,40)
        green_rgb = (0, 200, 120)
        red_rgb = (220, 40, 40)
        r = int(_interp(green_rgb[0], red_rgb[0], t))
        g = int(_interp(green_rgb[1], red_rgb[1], t))
        b = int(_interp(green_rgb[2], red_rgb[2], t))

        chunk_hex = f"#{r:02x}{g:02x}{b:02x}"
        
        # apply style
        self._apply_stylesheet(chunk_hex)

        try:
            # map temp into 0..100 percent using min/max
            if max_temp == min_temp:
                percent = 100.0
            else:
                percent = (temp - min_temp) / (max_temp - min_temp) * 100.0
        except Exception:
            percent = 100.0

        percent = max(0.0, min(100.0, percent))
        # Update the visual value
        self.setTargetValue(percent)

        return chunk_hex

    def set_percentage(self, percent: float):
        """Set progress 0..100 and update gradient color when choropleth is enabled.
        Low percent is red, high percent is green.
        """
        percent = max(0.0, min(100.0, float(percent)))
        # animate numeric value
        self.setTargetValue(percent)

        if self._choropleth:
            # compute gradient color between red and green (low=red, high=green)
            t = percent / 100.0
            red_rgb = (220, 40, 40)
            green_rgb = (0, 200, 120)
            r = int(_interp(red_rgb[0], green_rgb[0], t))
            g = int(_interp(red_rgb[1], green_rgb[1], t))
            b = int(_interp(red_rgb[2], green_rgb[2], t))
            chunk_hex = f"#{r:02x}{g:02x}{b:02x}"
            self._apply_stylesheet(chunk_hex)
        else:
            # keep solid color
            self._apply_stylesheet(self._solid_color)