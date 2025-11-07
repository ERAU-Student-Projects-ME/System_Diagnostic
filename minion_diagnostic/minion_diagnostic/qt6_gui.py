# TERMINOLOGY
# THE GRID:        the grid of squares that tiles can align to
# TILE:            one of the boxes with widgets in them
# (CHILD) WIDGET:  a graph, bar, chart, label, etc. inside a tile

import os
import sys
import json

from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QMainWindow, QSizePolicy, 
    QMenuBar, QTextEdit, QSplitter, QHBoxLayout, QTableWidget, QTableWidgetItem,
    QFileDialog, QHeaderView, QFrame
)
from PyQt6.QtCore import Qt, QSize, QPoint, QTimer, QEasingCurve, pyqtProperty, QPropertyAnimation
from PyQt6.QtGui import QColor, QPalette, QPainter, QPen, QAction
from widgets.custom_widgets_qt6 import SmoothProgressBar

# config
GRID_UNIT_SIZE = 50  # base size for a single grid cell (in px)

# consistent dark theme colors
THEME = {
    'menu_bg': '#1e1e1e',
    'menu_text': '#ccc',
    'window_bg': '#1e1e1e',
    'console_bg': '#1a1a1a',
    'console_text': '#aaa',
    'tile_label_color': '#ffffff',
    'table_text_color': '#fff'
}

class Tile(QWidget):
    def __init__(self, identifier, title, grid_w, grid_h, color, parent=None):
        super().__init__(parent)
        self.setObjectName(identifier)
        self.grid_width = grid_w
        self.grid_height = grid_h
        self.grid_x = 0
        self.grid_y = 0
        self.color_name = color

        # a list of dictionaries. each dict describes 1 child widget
        self.widgets_meta = []

        # makes sure the style sheet is applied
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.update_style()

        # layout for arranging widgets on tile
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)

        # header label
        if title:
            self.tile_header_label = QLabel(title)
            self.tile_header_label.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop)
            self.tile_header_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            self.tile_header_label.setFixedHeight(25)
            layout.addWidget(self.tile_header_label)

        # if the tile is being dragged
        self.dragging = False
        self.drag_start_mouse_pos = QPoint()
        self.drag_start_widget_pos = QPoint()

    def update_style(self):
        self.resize(int(self.grid_width * GRID_UNIT_SIZE), int(self.grid_height * GRID_UNIT_SIZE))
        self.setStyleSheet(f"""
            background-color: {self.color_name};
            border: 1px solid rgba(255, 255, 255, 140);
        """)

    def add_widget_meta(self, meta: dict):
        self.widgets_meta.append(meta)
        self.render_widgets_from_meta([meta])
    
    def render_widgets_from_meta(self, meta_list=None):
        if meta_list is None:
            meta_list = self.widgets_meta

        layout = self.layout()
        
        for meta in meta_list:
            id = meta.get('identifier', 'unknown')
            t = meta.get('type', '')
            if t == '':
                print(f"ERROR: widget {id} is missing a 'type' field:\n{meta}")
                continue
            
            if t == 'label':
                text = meta.get('text', '')
                lbl = QLabel(text, self)
                lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
                lbl.setStyleSheet(f"color: {THEME['tile_label_color']}; font-size: 16px;")
                layout.addWidget(lbl)

            elif t == 'progress':
                percent = meta.get('percent', 0)
                choropleth = bool(meta.get('choropleth', False))
                bar = SmoothProgressBar(self, choropleth=choropleth)
                bar.set_percentage(float(percent))
                layout.addWidget(bar)

            elif t == 'mode':
                mode_text = meta.get('value', 'none')
                lbl = QLabel(f"MODE: {mode_text}", self)
                lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
                lbl.setStyleSheet(f'color: {THEME["tile_label_color"]}; font-size: 36px; font-weight: bold;')
                layout.addWidget(lbl)

            elif t == 'indicator':
                state = bool(meta.get('state', False))
                light = QLabel(self)
                size_px = int(min(self.grid_width, self.grid_height) * 0.6 * GRID_UNIT_SIZE / max(1, GRID_UNIT_SIZE))
                size_px = max(40, min(120, size_px))
                light.setFixedSize(size_px, size_px)
                color = '#43a047' if state else '#e53935'
                light.setStyleSheet(f'background: {color}; border-radius: {size_px//2}px;')
                light.setAlignment(Qt.AlignmentFlag.AlignCenter)
                layout.addWidget(light, alignment=Qt.AlignmentFlag.AlignCenter)

            elif t == 'table':
                rows = meta.get('rows', [])
                cols = meta.get('cols', 2)
                table = QTableWidget(len(rows), cols, self)

                headers = meta.get('headers', '')
                if headers:
                    table.setHorizontalHeaderLabels(headers)
                
                # make table read-only and non-selectable
                table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
                table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
                table.setFocusPolicy(Qt.FocusPolicy.NoFocus)

                # hide cell background and outer-border
                table.setAlternatingRowColors(False)
                table.verticalHeader().setVisible(False)
                table.setShowGrid(True)
                table.setFrameShape(QFrame.Shape.NoFrame)
                table.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

                # fill width and height of tile
                table.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
                table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
                table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)

                # fill data
                for r, row in enumerate(rows):
                    for c, text in enumerate(row):
                        item = QTableWidgetItem(str(text))
                        table.setItem(r, c, item)

                # style header
                hdr = table.horizontalHeader()
                hdr.setSectionsClickable(False)
                layout.addWidget(table)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.dragging = True
            self.drag_start_mouse_pos = event.globalPosition().toPoint()
            self.drag_start_widget_pos = self.pos()
            self.raise_()
            event.accept()

    def mouseMoveEvent(self, event):
        if self.dragging:
            delta = event.globalPosition().toPoint() - self.drag_start_mouse_pos
            new_pos = self.drag_start_widget_pos + delta
            self.move(new_pos)
            event.accept()

    def mouseReleaseEvent(self, event):
        if self.dragging and event.button() == Qt.MouseButton.LeftButton:
            self.dragging = False
            self.parent().snap_tile_to_grid(self)
            event.accept()

class TileGrid(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setStyleSheet("background-color: #2e2e2e;")

        self.columns = 0
        self.rows = 0
        self.tiles = []

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        pen = QPen(QColor(40, 40, 40))
        pen.setWidth(1)
        painter.setPen(pen)

        # draw grid lines
        for col in range(1, self.columns):
            x = col * GRID_UNIT_SIZE
            painter.drawLine(x, 0, x, self.height())

        for row in range(1, self.rows):
            y = row * GRID_UNIT_SIZE
            painter.drawLine(0, y, self.width(), y)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._reflow_tiles()

    def _reflow_tiles(self):
        self.columns = max(1, self.width() // GRID_UNIT_SIZE)
        self.rows = max(1, self.height() // GRID_UNIT_SIZE)

        for tile in self.tiles:
            tile.update_style()
            new_x = max(0, min(tile.grid_x, self.columns - tile.grid_width))
            new_y = max(0, min(tile.grid_y, self.rows - tile.grid_height))
            tile.grid_x = new_x
            tile.grid_y = new_y
            tile.move(new_x * GRID_UNIT_SIZE, new_y * GRID_UNIT_SIZE)

        self.update()
        
    def to_dict(self):
        return {
            "tiles": [
                {
                    "identifier": t.objectName(),
                    "title": t.tile_header_label.text() if hasattr(t, 'tile_header_label') else t.objectName(),
                    "w": t.grid_width,
                    "h": t.grid_height,
                    "color": t.color_name,
                    "x": t.grid_x,
                    "y": t.grid_y,
                    "widgets": t.widgets_meta
                }
                for t in self.tiles
            ]
        }
        
    def save_layout(self, path):
        try:
            data = self.to_dict()
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
            print(f"Layout saved to {path}")
            return True
        except Exception as e:
            print(f"Failed to save layout: {e}")
            return False
        
    def load_layout(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            print(f"Failed to read layout file: {e}")
            return False
        
        self.clear_current()

        # creates tiles from data
        for spec in data.get('tiles', []):
            identifier = spec.get('identifier')
            if identifier is None:
                print(f"Warning: Tile missing identifier:\n{spec}")
                continue

            title = spec.get('title')
            width = spec.get('w', 2)
            height = spec.get('h', 2)
            color = spec.get('color', '#333333')
            x_pos = spec.get('x', 0)
            y_pos = spec.get('y', 0)

            tile = Tile(identifier, title, width, height, color, self)
            tile.grid_x = x_pos
            tile.grid_y = y_pos

            tile.widgets_meta = spec.get('widgets', [])
            tile.render_widgets_from_meta()

            self.tiles.append(tile)
            tile.show()

        self._reflow_tiles()
        print(f"Layout loaded from {path}")
        return True
    
    def clear_current(self):
        for tile in list(self.tiles):
            tile.setParent(None)
        self.tiles.clear()
        self.update()

    def snap_tile_to_grid(self, tile):
        new_grid_x = round(tile.x() / GRID_UNIT_SIZE)
        new_grid_y = round(tile.y() / GRID_UNIT_SIZE)

        new_grid_x = max(0, min(new_grid_x, self.columns - tile.grid_width))
        new_grid_y = max(0, min(new_grid_y, self.rows - tile.grid_height))
        
        # collision check
        for existing_tile in self.tiles:
            if existing_tile == tile:
                continue

            if (new_grid_x < existing_tile.grid_x + existing_tile.grid_width and
                new_grid_x + tile.grid_width > existing_tile.grid_x and
                new_grid_y < existing_tile.grid_y + existing_tile.grid_height and
                new_grid_y + tile.grid_height > existing_tile.grid_y):
                # collision detected (oh no), revert to original position
                new_grid_x = tile.grid_x
                new_grid_y = tile.grid_y
                break
            
        # update and final snap
        tile.grid_x = new_grid_x
        tile.grid_y = new_grid_y
        tile.move(new_grid_x * GRID_UNIT_SIZE, new_grid_y * GRID_UNIT_SIZE)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Minion Monitor")

        # MAIN LAYOUT STRUCTURE
        
        # nav bar
        self._setup_menu_bar()
        
        # 2. central widget (holds main window content)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # the grid
        self.grid = TileGrid()
        self.grid.setStyleSheet("background-color: #2e2e2e; border: 1px solid #555;")

        # console area
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setText("Successfully initialized. Console messages will show up here.")
        self.console.setStyleSheet("background-color: #1a1a1a; border-top: 1px solid #5aa5ee; color: #aaa;")

        # the draggable splitter between the grid and the console
        self.splitter = QSplitter(Qt.Orientation.Vertical)
        self.splitter.addWidget(self.grid)
        self.splitter.addWidget(self.console)

        self.setMinimumSize(QSize(600, 500))
        self.resize(1000, 800)

        self.splitter.setSizes([6 * GRID_UNIT_SIZE, 140])

        # apply the splitter
        main_layout = QVBoxLayout(central_widget)
        main_layout.setMenuBar(self.menuBar())
        main_layout.addWidget(self.splitter)
        main_layout.setContentsMargins(0, 0, 0, 0)

        self.setStyleSheet("QMainWindow { background-color: #1e1e1e; }")
        QTimer.singleShot(0, self._try_load_default_layout)

        # BOTTOM STATUS BAR
        status_bar_container = QWidget()
        status_layout = QHBoxLayout(status_bar_container)
        status_layout.setContentsMargins(0, 0, 0, 0)

        # pulsing status label
        self.status_pulse = QLabel('no connection')
        self.status_pulse.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_pulse.setStyleSheet('color: white; background: black; padding: 6px;')
        self.status_pulse.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)

        # ping indicator
        self.ping_label = QLabel('ping: -- ms')
        self.ping_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.ping_label.setStyleSheet('color: white; padding: 6px;')

        status_layout.addWidget(self.status_pulse)
        status_layout.addWidget(self.ping_label)

        main_layout.addWidget(status_bar_container)

        # smooth pulse animation
        self._status_connected = False
        self._pulse_anim = QPropertyAnimation(self, b"")
        self._pulse_anim.setDuration(1800)
        self._pulse_anim.setLoopCount(-1)
        self._pulse_anim.setEasingCurve(QEasingCurve.Type.InOutSine)
        self._pulse_anim.valueChanged.connect(self._update_pulse_color)
        self._pulse_anim.setKeyValueAt(0.0, 0.0)
        self._pulse_anim.setKeyValueAt(0.5, 1.0)
        self._pulse_anim.setKeyValueAt(1.0, 0.0)
        self._pulse_anim.start()

        self._apply_theme()

    def _update_pulse_color(self, value):
        if not self._status_connected:
            r1, g1, b1 = (0, 0, 0)
            r2, g2, b2 = (183, 28, 28)
            t = value
            r = int(r1 + (r2 - r1) * t)
            g = int(g1 + (g2 - g1) * t)
            b = int(b1 + (b2 - b1) * t)
            bg_hex = f'#{r:02x}{g:02x}{b:02x}'
            self.status_pulse.setStyleSheet(f'color: white; background: {bg_hex}; padding: 6px;')

    def _setup_menu_bar(self):
        menu_bar = QMenuBar()
        self.setMenuBar(menu_bar)

        # file menu
        file_menu = menu_bar.addMenu("File")
        open_action = QAction("Open Layout...", self)
        save_action = QAction("Save Layout...", self)
        file_menu.addAction(open_action)
        file_menu.addAction(save_action)

        # restore default
        restore_action = QAction('Restore Default Layout', self)
        restore_action.triggered.connect(self._try_load_default_layout)
        file_menu.addAction(restore_action)
        
        # wire up actions
        open_action.triggered.connect(self._open_layout)
        save_action.triggered.connect(self._save_layout)
    
    def _apply_theme(self):
        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Window, QColor(30, 30, 30))
        palette.setColor(QPalette.ColorRole.WindowText, QColor(200, 200, 200))
        palette.setColor(QPalette.ColorRole.Base, QColor(45, 45, 45))
        palette.setColor(QPalette.ColorRole.Text, QColor(200, 200, 200))
        palette.setColor(QPalette.ColorRole.Button, QColor(50, 50, 50))
        palette.setColor(QPalette.ColorRole.ButtonText, QColor(200, 200, 200))
        palette.setColor(QPalette.ColorRole.Highlight, QColor(42, 130, 218))
        palette.setColor(QPalette.ColorRole.HighlightedText, QColor(0, 0, 0))
        self.setPalette(palette)

        self.setStyleSheet(f"QMainWindow {{ background-color: {THEME['window_bg']}; }}")
        self.console.setStyleSheet(f"background-color: {THEME['console_bg']}; color: {THEME['console_text']}; border-top: 1px solid #5aa5ee;")
        self.menuBar().setStyleSheet(f"QMenuBar {{ background-color: {THEME['menu_bg']}; color: {THEME['menu_text']}; }}")
        self.status_pulse.setStyleSheet('color: white; padding: 6px; background: black;')
        self.ping_label.setStyleSheet('color: white; padding: 6px;')

    def set_connection_state(self, connected: bool, ping_ms: int | None = None):
        self._status_connected = bool(connected)
        if ping_ms is not None:
            try:
                self.ping_label.setText(f'ping: {ping_ms} ms')
            except:
                self.ping_label.setText('ping: -- ms')

        if self._status_connected:
            self.status_pulse.setStyleSheet('color: white; background: #2d7a2d; padding: 6px;')
            self._pulse_anim.stop()
        else:
            self._pulse_anim.start()

    def _try_load_default_layout(self):
        try:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            path = os.path.join(base_dir, 'default-layout.json')

            if not os.path.exists(path):
                self.console.append("No default layout found")
                return False
            
            return self.grid.load_layout(path)
                    
        except Exception as e:
            self.console.append(f"Error loading default layout: {e}")
            return False

    def _save_layout(self):
        path, _ = QFileDialog.getSaveFileName(
            self, 
            'Save Layout',
            os.path.join(os.path.dirname(os.path.abspath(__file__)), 'layouts'),
            'JSON Files (*.json)'
        )
        if path:
            self.grid.save_layout(path)

    def _open_layout(self):
        path, _ = QFileDialog.getOpenFileName(
            self,
            'Open Layout',
            os.path.join(os.path.dirname(os.path.abspath(__file__)), 'layouts'),
            'JSON Files (*.json)'
        )
        if path:
            self.grid.load_layout(path)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())