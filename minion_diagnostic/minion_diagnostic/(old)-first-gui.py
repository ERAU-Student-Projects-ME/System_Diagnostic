# TERMINOLOGY
# GRID:   the grid of SQUARES that tiles can align to
# TILE:   one of the boxes with information in them
# WIDGET: a graph, bar, chart, label, etc.

import sys
import json
import os
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QMainWindow, QSizePolicy, 
    QMenuBar, QAction, QTextEdit, QSplitter, QHBoxLayout, QTableWidget, QTableWidgetItem,
    QFileDialog, QHeaderView, QFrame
)
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import Qt, QSize, QPoint, QTimer, QEasingCurve, pyqtProperty, QPropertyAnimation
from PyQt5.QtGui import QColor, QPalette, QPainter, QPen
from widgets.custom_widgets import SmoothProgressBar

# config
SCALE_FACTOR = 1.0   # controls the scale of the grid/tiles (1 = normal size)
GRID_UNIT_SIZE = 50  # base size for a single grid cell (in pixels)
CORNER_RADIUS = 5   # radius for rounding the corners of the tiles

# theme values
THEME = {
    'dark': True,
    'menu_bg': '#1e1e1e',
    'menu_text': '#ccc',
    'window_bg': '#1e1e1e',
    'console_bg': '#1a1a1a',
    'console_text': '#aaa',
    'tile_label_color': '#ffffff',
    'table_text_color': '#fff'
}

class Tile(QWidget):
    def __init__(self, name, grid_w, grid_h, color, parent=None):
        super().__init__(parent)
        self.setObjectName(name)
        self.grid_width = grid_w
        self.grid_height = grid_h
        self.grid_x = 0
        self.grid_y = 0
        self.color_name = color

        # metadata for widgets inside this tile (used for saving/loading layouts)
        # format is a dict like {"type": "table", ...}
        self.widgets_meta = []
        # ensure stylesheet backgrounds are applied
        self.setAttribute(Qt.WA_StyledBackground, True)

        self.update_size_and_style()

    # add a layout for child content inside the tile
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        # header label (kept as attribute; show just the tile title)
        if name:
            self.header_label = QLabel(name)
            # top-align all titles and keep them small so content centers below
            self.header_label.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
            self.header_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            self.header_label.setFixedHeight(28)
            layout.addWidget(self.header_label)
        
        self.dragging = False
        self.drag_start_mouse_pos = QPoint()
        self.drag_start_widget_pos = QPoint()
        
    def update_size_and_style(self):
        scaled_grid_size = GRID_UNIT_SIZE * SCALE_FACTOR
        
        self.resize(int(self.grid_width * scaled_grid_size), 
                    int(self.grid_height * scaled_grid_size))

        # Scope styles to this tile via its objectName so children can be made transparent
        # increase border visibility compared to previous low-alpha value
        self.setStyleSheet(f"""
            QWidget#{self.objectName()} {{
                background-color: {self.color_name};
                border-radius: {CORNER_RADIUS}px;
                border: 1px solid rgba(255, 255, 255, 140);
            }}
            QWidget#{self.objectName()} QLabel {{
                color: {THEME['tile_label_color']};
                font-weight: bold;
                background: transparent;
            }}
            QWidget#{self.objectName()} QTableWidget,
            QWidget#{self.objectName()} QWidget {{
                background: transparent;
                border: none;
            }}
        """)

    def clear_content_widgets(self):
        """Remove all content widgets from the tile layout, keep header_label."""
        layout = self.layout()
        # remove items after the header (assume header is first)
        while layout.count() > 1:
            item = layout.takeAt(1)
            w = item.widget()
            if w is not None:
                try:
                    w.setParent(None)
                except Exception:
                    pass

    def add_widget_meta(self, meta: dict):
        """Append widget metadata and render it into the tile."""
        self.widgets_meta.append(meta)
        self.render_widgets_from_meta([meta])

    def render_widgets_from_meta(self, meta_list=None):
        """Render widgets in this tile from metadata entries.
        meta_list: list of metadata dicts. If None, uses self.widgets_meta to render all.
        Supported types: 'label' (text), 'table' (rows: list of [col1, col2,..]),
        'progress' (choropleth bool, percent number, data_stream str).
        """
        if meta_list is None:
            meta_list = self.widgets_meta

        # clear existing content widgets
        self.clear_content_widgets()
        layout = self.layout()

        for meta in meta_list:
            t = meta.get('type', 'label')
            if t == 'label':
                text = meta.get('text', '')
                lbl = QLabel(text, self)
                lbl.setAlignment(Qt.AlignCenter)
                lbl.setStyleSheet(f'color: {THEME["tile_label_color"]}; background: transparent; font-size: 16px;')
                layout.addWidget(lbl)
            elif t == 'table':
                rows = meta.get('rows', [])
                cols = meta.get('cols', 2)
                table = QTableWidget(len(rows), cols, self)
                
                headers = meta.get('headers')
                if headers:
                    table.setHorizontalHeaderLabels(headers)

                # Make table read-only and non-selectable to avoid header focus/selection issues
                table.setEditTriggers(QTableWidget.NoEditTriggers)
                table.setSelectionMode(QTableWidget.NoSelection)
                table.setFocusPolicy(Qt.NoFocus)
                # show only separators (grid lines) but keep cell backgrounds transparent
                table.setAlternatingRowColors(False)
                table.verticalHeader().setVisible(False)
                table.setShowGrid(True)
                table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

                # Fill rows
                for r, row in enumerate(rows):
                    for c, val in enumerate(row[:cols]):
                        item = QTableWidgetItem(str(val))
                        # enable but not editable
                        item.setFlags(Qt.ItemIsEnabled)
                        table.setItem(r, c, item)

                # Tweak header appearance and disable header clicks/selection
                hdr = table.horizontalHeader()
                hdr.setSectionsClickable(False)
                try:
                    hdr.setHighlightSections(False)
                except Exception:
                    pass
                # Make columns stretch to fill the available tile width
                try:
                    hdr.setSectionResizeMode(QHeaderView.Stretch)
                    hdr.setStretchLastSection(True)
                except Exception:
                    pass

                # remove outer frame, keep gridlines as separators and transparent cells
                try:
                    table.setFrameShape(QFrame.NoFrame)
                except Exception:
                    pass

                table.setStyleSheet(
                    f"QTableWidget {{ background: transparent; border: none; color: {THEME['table_text_color']}; gridline-color: rgba(255,255,255,30); }}"
                    "QTableWidget::item { background: transparent; }"
                )
                table.horizontalHeader().setStyleSheet(f"QHeaderView::section {{ background: transparent; color: {THEME['table_text_color']}; font-weight: bold; padding: 4px; }}")
                layout.addWidget(table)
            elif t == 'progress':
                percent = meta.get('percent', None)
                choropleth = bool(meta.get('choropleth', False))
                bar = SmoothProgressBar(self, choropleth=choropleth)
                if percent is not None:
                    try:
                        bar.set_percentage(float(percent))
                    except Exception:
                        pass
                # ensure percent text is white regardless of chunk color
                try:
                    last = getattr(bar, '_last_chunk_color', None)
                    if last:
                        bar._apply_stylesheet(last, '#ffffff')
                except Exception:
                    pass
                layout.addWidget(bar)
            elif t == 'indicator':
                # meta: {'type':'drone', 'state': True/False}
                state = bool(meta.get('state', False))
                circle = QLabel(self)
                size_px = int(min(self.grid_width, self.grid_height) * 0.6 * (GRID_UNIT_SIZE * SCALE_FACTOR) / max(1, GRID_UNIT_SIZE))
                size_px = max(40, min(120, size_px))
                circle.setFixedSize(size_px, size_px)
                color = '#43a047' if state else '#e53935'
                circle.setStyleSheet(f'background: {color}; border-radius: {size_px//2}px;')
                circle.setAlignment(Qt.AlignCenter)
                layout.addWidget(circle, alignment=Qt.AlignCenter)
            elif t == 'mode':
                # meta: {'type':'mode','input': 'none'}
                mode_text = meta.get('input', 'none')
                
                # hide the header/title for this tile (redundant)
                self.header_label.setVisible(False)
                
                lbl = QLabel(f'MODE: {mode_text}', self)
                lbl.setAlignment(Qt.AlignCenter)
                lbl.setStyleSheet(f'color: {THEME["tile_label_color"]}; background: transparent; font-size: 36px; font-weight: bold;')
                layout.addWidget(lbl)
            else:
                # unknown type: show repr
                lbl = QLabel(repr(meta), self)
                lbl.setStyleSheet('color: #fff; background: transparent;')
                layout.addWidget(lbl)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.dragging = True
            self.setStyleSheet(self.styleSheet() + "border: 2px solid #5aa5ee;") 
            self.drag_start_mouse_pos = event.globalPos()
            self.drag_start_widget_pos = self.pos()
            self.raise_() 
            event.accept()

    def mouseMoveEvent(self, event):
        if self.dragging:
            delta = event.globalPos() - self.drag_start_mouse_pos
            new_pos = self.drag_start_widget_pos + delta
            self.move(new_pos)
            event.accept()

    def mouseReleaseEvent(self, event):
        if self.dragging and event.button() == Qt.LeftButton:
            self.dragging = False
            self.setStyleSheet(self.styleSheet().replace("border: 2px solid #5aa5ee;", f"border: 1px solid rgba(255, 255, 255, 50);"))
            self.parent().snap_tile_to_grid(self)
            event.accept()

# --- 2. Grid Container Widget (Minimal Changes) ---
class TileGrid(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(QSize(4 * GRID_UNIT_SIZE, 3 * GRID_UNIT_SIZE))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet("background-color: #2e2e2e;")
        
        self.columns = 0
        self.rows = 0
        self.tiles = [] 
        # Do not initialize tiles here; the layout should be blank until a layout file is loaded.
        # self._initialize_tiles()
        
    @property
    def current_grid_size(self):
        return int(GRID_UNIT_SIZE * SCALE_FACTOR)

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        pen = QPen(QColor(40, 40, 40)) 
        pen.setWidth(1)
        painter.setPen(pen)

        for col in range(1, self.columns):
            x = col * self.current_grid_size
            painter.drawLine(x, 0, x, self.height())

        for row in range(1, self.rows):
            y = row * self.current_grid_size
            painter.drawLine(0, y, self.width(), y)
            
    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._reflow_tiles()

    def _reflow_tiles(self):
        """Recalculate grid columns/rows and reposition tiles. Safe to call without a QResizeEvent."""
        scaled_size = self.current_grid_size
        self.columns = max(1, self.width() // scaled_size)
        self.rows = max(1, self.height() // scaled_size)

        for tile in self.tiles:
            tile.update_size_and_style()

            new_x = max(0, min(tile.grid_x, self.columns - tile.grid_width))
            new_y = max(0, min(tile.grid_y, self.rows - tile.grid_height))

            tile.grid_x = new_x
            tile.grid_y = new_y
            tile.move(new_x * scaled_size, new_y * scaled_size)

        self.update()

    
    # --- Layout serialization ---
    def to_dict(self):
        """Return a serializable representation of the current layout."""
        return {
            "grid_unit_size": GRID_UNIT_SIZE,
            "tiles": [
                {
                    "name": t.objectName(),
                    "title": t.header_label.text() if hasattr(t, 'header_label') else t.objectName(),
                    "w": t.grid_width,
                    "h": t.grid_height,
                    "color": t.color_name,
                    "x": t.grid_x,
                    "y": t.grid_y,
                    "widgets": t.widgets_meta if hasattr(t, 'widgets_meta') else [],
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

        # Remove existing tiles
        for t in list(self.tiles):
            try:
                t.setParent(None)
            except Exception:
                pass
        self.tiles.clear()
        # Create tiles from data
        for spec in data.get('tiles', []):
            name = spec.get('name', spec.get('title', 'Tile'))
            w = spec.get('w', 2)
            h = spec.get('h', 2)
            color = spec.get('color', '#888')
            x = spec.get('x', 0)
            y = spec.get('y', 0)

            tile = Tile(name, w, h, color, self)
            tile.grid_x = x
            tile.grid_y = y
            # set header title if provided
            try:
                tile.header_label.setText(spec.get('title', name))
            except Exception:
                pass
            # restore widget metadata and render
            widgets = spec.get('widgets', [])
            tile.widgets_meta = widgets
            try:
                tile.render_widgets_from_meta()
            except Exception:
                pass

            self.tiles.append(tile)
            tile.show()

        # Force layout recalculation
        self._reflow_tiles()
        print(f"Layout loaded from {path}")
        return True

    def clear_layout(self):
        for t in list(self.tiles):
            try:
                t.setParent(None)
            except Exception:
                pass
        self.tiles.clear()
        self.update()
            
    def snap_tile_to_grid(self, tile):
        scaled_size = self.current_grid_size
        
        new_grid_x = round(tile.x() / scaled_size)
        new_grid_y = round(tile.y() / scaled_size)

        new_grid_x = max(0, min(new_grid_x, self.columns - tile.grid_width))
        new_grid_y = max(0, min(new_grid_y, self.rows - tile.grid_height))

        # Collision Check 
        for existing_tile in self.tiles:
            if existing_tile == tile: continue
            
            if (new_grid_x < existing_tile.grid_x + existing_tile.grid_width and
                new_grid_x + tile.grid_width > existing_tile.grid_x and
                new_grid_y < existing_tile.grid_y + existing_tile.grid_height and
                new_grid_y + tile.grid_height > existing_tile.grid_y):
                
                tile.move(tile.grid_x * scaled_size, tile.grid_y * scaled_size)
                return

        # Update and Final snap
        tile.grid_x = new_grid_x
        tile.grid_y = new_grid_y
        tile.move(new_grid_x * scaled_size, new_grid_y * scaled_size)


# --- 3. Main Application Setup (Heavily Revised) ---
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Minion Monitor")
        self._apply_theme()
        
        # --- Main Layout Structure ---
        # 1. Menu Bar (Top Navigation)
        self._setup_menu_bar()
        
        # 2. Central Widget Container (to hold the splitter)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 3. Vertical Splitter (Divides Grid area from Console area)
        self.splitter = QSplitter(Qt.Vertical)
        
        # 4. Tile Grid (The main work area)
        self.grid = TileGrid()
        self.grid.setStyleSheet("background-color: #2e2e2e; border: 1px solid #555;") # Added border for separation
        
        # 5. Console/Control Bar (The resizable bottom area)
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setText("Successfully initialized. Console message will show up here.")
        # Do not fix height so the splitter can resize this area
        self.console.setStyleSheet("background-color: #1a1a1a; border-top: 1px solid #5aa5ee; color: #aaa;")
        
        # 6. Add Grid and Console to Splitter
        self.splitter.addWidget(self.grid)
        self.splitter.addWidget(self.console)

        # set the initial window size first so splitter calculations are correct
        self.setMinimumSize(QSize(600, 500))
        self.resize(1000, 800) # Larger initial size for better layout visualization

        # Set initial sizes for the splitter items so the grid area maps to 12x6 cells
        # Grid height = 6 * GRID_UNIT_SIZE = 600, console ~= 260 px on startup
        self.splitter.setSizes([6 * GRID_UNIT_SIZE, 140])

        # 7. Apply Splitter to Central Widget Layout
        main_layout = QVBoxLayout(central_widget)
        main_layout.setMenuBar(self.menuBar()) # Add the menu bar to the layout
        main_layout.addWidget(self.splitter)
        main_layout.setContentsMargins(0, 0, 0, 0) # Remove margin around the central widget

        self.setStyleSheet("QMainWindow { background-color: #1e1e1e; }")
        # Defer loading the default layout until after the window is shown so
        # the grid has a valid size & can reflow tiles correctly.
        QTimer.singleShot(0, self._try_load_default_layout)

        # --- Bottom status bar (below console) ---
        status_bar_container = QWidget()
        status_layout = QHBoxLayout(status_bar_container)
        status_layout.setContentsMargins(0, 0, 0, 0)

        # pulsing center label
        self.status_pulse = QLabel('no connection')
        self.status_pulse.setAlignment(Qt.AlignCenter)
        self.status_pulse.setStyleSheet('color: white; background: black; padding: 6px;')
        self.status_pulse.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

        # ping indicator on right
        self.ping_label = QLabel('ping: -- ms')
        self.ping_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.ping_label.setStyleSheet('color: white; padding: 6px;')

        status_layout.addWidget(self.status_pulse)
        status_layout.addWidget(self.ping_label)

        main_layout.addWidget(status_bar_container)

        # smooth pulse
        self._status_connected = False
        self._pulse_opacity = 1.0
        self._pulse_anim = QPropertyAnimation(self, b"pulseOpacity", self)
        self._pulse_anim.setDuration(2000)
        self._pulse_anim.setLoopCount(-1)
        self._pulse_anim.setEasingCurve(QEasingCurve.InOutQuad)
        
        self._pulse_anim.setKeyValueAt(0.0, 0.0)
        self._pulse_anim.setKeyValueAt(0.5, 1.0)
        self._pulse_anim.setKeyValueAt(1.0, 0.0)
        
        self._pulse_anim.start()

    def _setup_menu_bar(self):
        """Creates the Nav Bar at the top."""
        menu_bar = QMenuBar()
        self.setMenuBar(menu_bar)

        # File Menu
        file_menu = menu_bar.addMenu("File")
        open_action = QAction("Open Layout...", self)
        save_action = QAction("Save Layout...", self)
        file_menu.addAction(open_action)
        file_menu.addAction(save_action)

        # Restore default and clear layout actions
        restore_action = QAction('Restore Default Layout', self)
        restore_action.triggered.connect(self._restore_default_layout)
        file_menu.addAction(restore_action)

        clear_action = QAction('Clear Layout', self)
        clear_action.triggered.connect(self._clear_layout)
        file_menu.addAction(clear_action)

        # View Menu
        view_menu = menu_bar.addMenu("View")
        reset_window_size_action = QAction("Restore Window Size", self)
        zoom_in_action = QAction("Zoom In", self)
        zoom_out_action = QAction("Zoom Out", self)

        view_menu.addAction(reset_window_size_action)
        view_menu.addAction(zoom_in_action)
        view_menu.addAction(zoom_out_action)
        # Theme toggle
        toggle_theme_action = QAction('Toggle Dark/Light', self)
        toggle_theme_action.triggered.connect(self._toggle_theme)
        view_menu.addAction(toggle_theme_action)

        # wire the reset action (handler may reference self.splitter which is created later)
        reset_window_size_action.triggered.connect(self._reset_window_size)
        # wire save/load actions
        open_action.triggered.connect(self._open_layout)
        save_action.triggered.connect(self._save_layout)

        # Apply menu styling using THEME values
        menu_bar.setStyleSheet(f"""
            QMenuBar {{
                background-color: {THEME['menu_bg']}; 
                color: {THEME['menu_text']}; 
                border-bottom: 1px solid #333;
            }}
            QMenuBar::item:selected {{
                background-color: #2a2a2a;
            }}
            QMenu {{
                background-color: #2a2a2a; 
                border: 1px solid #444;
            }}
            QMenu::item {{
                color: {THEME['menu_text']};
            }}
            QMenu::item:selected {{ 
                background-color: #424242; 
            }}
        """)

    def _apply_theme(self):
        """Apply colors and palette according to THEME dict."""
        palette = QPalette()
        if THEME.get('dark', True):
            # dark palette
            palette.setColor(QPalette.Window, QColor(30, 30, 30))
            palette.setColor(QPalette.WindowText, QColor(200, 200, 200))
            palette.setColor(QPalette.Base, QColor(45, 45, 45))
            palette.setColor(QPalette.Text, QColor(200, 200, 200))
            palette.setColor(QPalette.Button, QColor(50, 50, 50))
            palette.setColor(QPalette.ButtonText, QColor(200, 200, 200))
            palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
            palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
        else:
            # light palette
            palette.setColor(QPalette.Window, QColor(250, 250, 250))
            palette.setColor(QPalette.WindowText, QColor(20, 20, 20))
            palette.setColor(QPalette.Base, QColor(245, 245, 245))
            palette.setColor(QPalette.Text, QColor(20, 20, 20))
            palette.setColor(QPalette.Button, QColor(240, 240, 240))
            palette.setColor(QPalette.ButtonText, QColor(20, 20, 20))
            palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
            palette.setColor(QPalette.HighlightedText, QColor(255, 255, 255))
        self.setPalette(palette)

        # update main window background and console/menu colors
        self.setStyleSheet(f"QMainWindow {{ background-color: {THEME['window_bg']}; }}")
        try:
            self.console.setStyleSheet(f"background-color: {THEME['console_bg']}; color: {THEME['console_text']}; border-top: 1px solid #5aa5ee;")
        except Exception:
            pass
        try:
            self.menuBar().setStyleSheet(f"QMenuBar {{ background-color: {THEME['menu_bg']}; color: {THEME['menu_text']}; }}")
        except Exception:
            pass
        try:
            # update status bar colors to match theme
            self.status_pulse.setStyleSheet('color: white; padding: 6px; background: black;')
            self.ping_label.setStyleSheet('color: white; padding: 6px;')
        except Exception:
            pass

    def _toggle_theme(self):
        """Flip between dark and light themes and reapply UI styles."""
        THEME['dark'] = not THEME.get('dark', True)
        if THEME['dark']:
            THEME.update({
                'menu_bg': '#1e1e1e', 'menu_text': '#ccc', 'window_bg': '#1e1e1e',
                'console_bg': '#1a1a1a', 'console_text': '#aaa',
                'tile_label_color': '#ffffff', 'table_text_color': '#fff'
            })
        else:
            THEME.update({
                'menu_bg': '#f6f6f6', 'menu_text': '#222', 'window_bg': '#ffffff',
                'console_bg': '#f0f0f0', 'console_text': '#222',
                'tile_label_color': '#111111', 'table_text_color': '#111111'
            })

        # Apply palette and styles
        self._apply_theme()

        # Reapply tile styles and content widgets so colors update
        try:
            for t in self.grid.tiles:
                try:
                    t.update_size_and_style()
                    # re-render widgets from metadata to apply new text colors
                    t.render_widgets_from_meta()
                except Exception:
                    pass
        except Exception:
            pass


    def _apply_dark_theme(self):
        palette = QPalette()
        dark_mode_colors = {
            QPalette.Window: QColor(30, 30, 30),
            QPalette.WindowText: QColor(200, 200, 200),
            QPalette.Base: QColor(45, 45, 45),
            QPalette.Text: QColor(200, 200, 200),
            QPalette.Button: QColor(50, 50, 50),
            QPalette.ButtonText: QColor(200, 200, 200),
            QPalette.Highlight: QColor(42, 130, 218),
            QPalette.HighlightedText: QColor(0, 0, 0),
        }
        for role, color in dark_mode_colors.items():
            palette.setColor(role, color)
        self.setPalette(palette)

    def _reset_window_size(self):
        """Restore the main window to the default 1200x800 and reasonable splitter sizes."""
        self.resize(1000, 800)
        # Set splitter so grid area is 6 rows tall (6 * GRID_UNIT_SIZE)
        if hasattr(self, 'splitter'):
            grid_h = 555
            bottom = max(100, self.height() - grid_h)
            self.splitter.setSizes([grid_h, bottom])

    # Status bar helpers
    def _pulse_tick(self):
        # kept for compatibility; animation handles pulsing now
        if self._status_connected:
            self.status_pulse.setStyleSheet('color: white; background: #2d7a2d; padding: 6px;')
        else:
            # show a color according to current opacity
            alpha = int(40 + 215 * self._pulse_opacity)
            # interpolate between black and red using opacity
            if alpha < 128:
                bg = 'black'
            else:
                bg = '#b71c1c'
            self.status_pulse.setStyleSheet(f'color: white; background: {bg}; padding: 6px;')

    def set_connection_state(self, connected: bool, ping_ms: int | None = None):
        self._status_connected = bool(connected)
        if ping_ms is not None:
            try:
                self.ping_label.setText(f'ping: {int(ping_ms)} ms')
            except Exception:
                self.ping_label.setText('ping: -- ms')
        # adjust animation behavior immediately
        if self._status_connected:
            # stop pulsing and show solid green
            try:
                self._pulse_anim.pause()
            except Exception:
                pass
            self.status_pulse.setStyleSheet('color: white; background: #2d7a2d; padding: 6px;')
        else:
            try:
                self._pulse_anim.resume()
            except Exception:
                pass
            # ensure tick applies one frame
            self._pulse_tick()

    # --- Tile update helpers / live-data API ---
    def _find_tile_by_name(self, name: str):
        for t in self.grid.tiles:
            if t.objectName() == name or (hasattr(t, 'header_label') and t.header_label.text() == name):
                return t
        return None

    def update_indicator(self, tile_name: str, state: bool):
        t = self._find_tile_by_name(tile_name)
        if not t:
            return False
        t.widgets_meta = [{'type': 'indicator', 'state': bool(state)}]
        try:
            t.render_widgets_from_meta()
        except Exception:
            pass
        return True

    def update_mode(self, tile_name: str, text: str):
        t = self._find_tile_by_name(tile_name)
        if not t:
            return False
        t.widgets_meta = [{'type': 'mode', 'value': str(text)}]
        try:
            t.render_widgets_from_meta()
        except Exception:
            pass
        return True

    def update_progress(self, tile_name: str, percent: float):
        t = self._find_tile_by_name(tile_name)
        if not t:
            return False
        t.widgets_meta = [{'type': 'progress', 'percent': float(percent)}]
        try:
            t.render_widgets_from_meta()
        except Exception:
            pass
        return True

    def update_table(self, tile_name: str, headers, rows):
        t = self._find_tile_by_name(tile_name)
        if not t:
            return False
        t.widgets_meta = [{'type': 'table', 'headers': headers, 'rows': rows}]
        try:
            t.render_widgets_from_meta()
        except Exception:
            pass
        return True

    def update_from_system_status(self, msg):
        """
        Adapter for SystemStatus-like messages. Maps common fields to tiles.
        Extend as needed for your actual message types.
        """
        try:
            if hasattr(msg, 'battery_percentage'):
                self.update_progress('Battery', getattr(msg, 'battery_percentage', 0.0))
            if hasattr(msg, 'drone_online'):
                self.update_indicator('drone', getattr(msg, 'drone_online', False))
            if hasattr(msg, 'mode'):
                self.update_mode('Mode', getattr(msg, 'mode', 'none'))
            # sensors: expect list of simple objects or tuples
            if hasattr(msg, 'sensors') and isinstance(getattr(msg, 'sensors'), (list, tuple)):
                rows = []
                for s in msg.sensors:
                    if isinstance(s, (list, tuple)):
                        rows.append([str(s[0]), str(s[1]) if len(s) > 1 else ''])
                    else:
                        rows.append([getattr(s, 'name', str(s)), getattr(s, 'status', '')])
                self.update_table('Sensors', ['Sensor', 'Status'], rows)
        except Exception:
            pass

    # Animation property used by QPropertyAnimation for smooth pulsing
    def getPulseOpacity(self):
        return self._pulse_opacity

    def setPulseOpacity(self, v):
        try:
            self._pulse_opacity = float(v)
        except Exception:
            self._pulse_opacity = 1.0
        # update visible color
        if not self._status_connected:
            # compute smooth blend between black (0,0,0) and red (183,28,28 / #b71c1c)
            r1, g1, b1 = (0, 0, 0)
            r2, g2, b2 = (183, 28, 28)
            t = max(0.0, min(1.0, self._pulse_opacity))
            r = int(r1 + (r2 - r1) * t)
            g = int(g1 + (g2 - g1) * t)
            b = int(b1 + (b2 - b1) * t)
            bg_hex = f'#{r:02x}{g:02x}{b:02x}'
            self.status_pulse.setStyleSheet(f'color: white; background: {bg_hex}; padding: 6px;')

    pulseOpacity = pyqtProperty(float, getPulseOpacity, setPulseOpacity)

    # --- Layout menu handlers ---
    def _default_layout_path(self):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(base_dir, 'default-layout.json')

    def _restore_default_layout(self):
        path = self._default_layout_path()
        if not os.path.exists(path):
            QMessageBox.critical(self, 'Restore Failed', 'default layout file not found')
            return
        try:
            ok = self.grid.load_layout(path)
            if not ok:
                QMessageBox.critical(self, 'Restore Failed', 'Failed to load default layout')
        except Exception as e:
            QMessageBox.critical(self, 'Restore Failed', f'Failed to load default layout: {e}')

    def _clear_layout(self):
        resp = QMessageBox.question(self, 'Clear Layout', 'Clear the current layout? This cannot be undone.', QMessageBox.Yes | QMessageBox.No)
        if resp == QMessageBox.Yes:
            self.grid.clear_layout()
            self.console.append('Layout cleared')
        else:
            self.console.append('Clear layout cancelled')

    def _ensure_default_layout(self):
        path = self._default_layout_path()
        # If default layout is missing or invalid, we do NOT auto-create a sample layout.
        # The requirement is to show an error dialog and keep the layout blank.
        if not os.path.exists(path):
            QMessageBox.critical(self, 'Default Layout Missing', 'default layout file not found')
            return False

        # try to validate JSON
        try:
            with open(path, 'r', encoding='utf-8') as f:
                txt = f.read().strip()
            if not txt:
                QMessageBox.critical(self, 'Default Layout Missing', 'default layout file not found')
                return False
            json.loads(txt)
            return True
        except Exception:
            QMessageBox.critical(self, 'Default Layout Error', 'default layout file is invalid')
            return False

    def _try_load_default_layout(self):
        """Called after window show to try loading default layout if valid."""
        try:
            ok = self._ensure_default_layout()
            if ok:
                default_path = self._default_layout_path()
                if os.path.exists(default_path):
                    loaded = self.grid.load_layout(default_path)
                    if not loaded:
                        self.console.append('Failed to load default layout on startup')
                    else:
                        self.console.append(f'Loaded default layout from {default_path}')
        except Exception as e:
            # Avoid crashing on startup; report to console
            try:
                self.console.append(f'Error loading default layout: {e}')
            except Exception:
                pass

    def _save_layout(self):
        # Prompt user to choose a file to save layout
        path, _ = QFileDialog.getSaveFileName(self, 'Save Layout', self._default_layout_path(), 'JSON Files (*.json)')
        if path:
            ok = self.grid.save_layout(path)
            if ok:
                self.console.append(f"Saved layout to {path}")

    def _open_layout(self):
        path, _ = QFileDialog.getOpenFileName(self, 'Open Layout', os.path.dirname(self._default_layout_path()), 'JSON Files (*.json)')
        if path:
            ok = self.grid.load_layout(path)
            if ok:
                self.console.append(f"Loaded layout from {path}")


if __name__ == '__main__':
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)
    
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())