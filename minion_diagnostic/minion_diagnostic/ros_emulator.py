"""ros_emulator.py

Simple local emulator that generates randomized updates for tiles/widgets.

This is a minimal stand-in for a future ROS2-backed data source. It provides:
- DataEmulator: a thread-based generator with callback registration.
- QtEmulator: a Qt-friendly QObject (QTimer) that emits `update` signals when
  PyQt6 is available. This makes it safe and easy to connect directly to UI
  slots or functions in the main thread.

Usage (non-Qt):
    from ros_emulator import DataEmulator

    def on_update(payload):
        # payload is a dict containing 'tiles' -> list[...] with widgets meta
        print(payload)

    em = DataEmulator(update_interval=0.5)
    em.register_callback(on_update)
    em.start()

    # stop with em.stop()

Usage (Qt):
    from ros_emulator import QtEmulator
    em = QtEmulator(interval_ms=500)
    em.update.connect(your_slot)
    em.start()

The payload format intentionally mirrors the tile layout json used by the
application: { 'tiles': [ { 'identifier': ..., 'widgets': [ {..}, ... ] }, ... ] }
"""

from __future__ import annotations

import random
import threading
import time
import logging
from typing import Callable, Dict, List, Optional, Any

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


class DataEmulator:
    """Threaded emulator that periodically calls registered callbacks with a
    generated payload describing tiles and their child widgets.

    Callbacks receive a single dict argument: { 'tiles': [ ... ] }
    """

    def __init__(self, update_interval: float = 0.5, seed: Optional[int] = None,
                 tile_ids: Optional[List[str]] = None):
        self.update_interval = float(update_interval)
        self._callbacks: List[Callable[[Dict[str, Any]], None]] = []
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        self._rand = random.Random(seed)

        # default tile identifiers if none provided
        if tile_ids is None:
            self.tile_ids = [
                "tile_cpu",
                "tile_mem",
                "tile_temp",
                "tile_mode",
                "tile_table",
            ]
        else:
            self.tile_ids = list(tile_ids)

    def register_callback(self, fn: Callable[[Dict[str, Any]], None]) -> None:
        with self._lock:
            if fn not in self._callbacks:
                self._callbacks.append(fn)

    def unregister_callback(self, fn: Callable[[Dict[str, Any]], None]) -> None:
        with self._lock:
            if fn in self._callbacks:
                self._callbacks.remove(fn)

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        logger.info("DataEmulator started (interval=%s)", self.update_interval)

    def stop(self, timeout: float = 1.0) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout)
            self._thread = None
        logger.info("DataEmulator stopped")

    def _run_loop(self) -> None:
        while self._running:
            payload = self.generate_update()
            # call callbacks with copy-safe semantics
            with self._lock:
                cbs = list(self._callbacks)

            for cb in cbs:
                try:
                    cb(payload)
                except Exception:
                    logger.exception("Emulator callback raised an exception")

            time.sleep(self.update_interval)

    def generate_update(self) -> Dict[str, Any]:
        """Construct a randomized payload for all configured tiles.

        The structure is intentionally simple and mirrors the tile/widget meta
        the UI expects. Consumers may adapt the payload to their needs.
        """
        tiles_out: List[Dict[str, Any]] = []

        for tid in self.tile_ids:
            # produce widgets based on tile name heuristics
            widgets = []
            if "cpu" in tid:
                widgets.append({
                    "identifier": f"{tid}_usage",
                    "type": "progress",
                    "percent": round(self._rand.uniform(5.0, 95.0), 1),
                    "choropleth": True,
                })
                widgets.append({
                    "identifier": f"{tid}_mode",
                    "type": "mode",
                    "value": self._rand.choice(["IDLE", "RUN", "WAIT"]),
                })

            elif "mem" in tid:
                widgets.append({
                    "identifier": f"{tid}_usage",
                    "type": "progress",
                    "percent": round(self._rand.uniform(12.0, 92.0), 1),
                    "choropleth": False,
                })

            elif "temp" in tid:
                # use the temperature helper style
                temp = round(self._rand.uniform(25.0, 95.0), 1)
                widgets.append({
                    "identifier": f"{tid}_temp",
                    "type": "progress",
                    "percent": temp,  # consumer can treat as temperature or percent
                    "choropleth": True,
                })
                widgets.append({
                    "identifier": f"{tid}_fan",
                    "type": "indicator",
                    "state": self._rand.choice([True, False]),
                })

            elif "mode" in tid:
                widgets.append({
                    "identifier": f"{tid}_big",
                    "type": "mode",
                    "value": self._rand.choice(["AUTO", "MANUAL", "SAFE"]),
                })

            elif "table" in tid:
                rows = []
                for r in range(3):
                    rows.append([f"row{r+1}", round(self._rand.uniform(0, 100), 2)])
                widgets.append({
                    "identifier": f"{tid}_tbl",
                    "type": "table",
                    "rows": rows,
                    "cols": 2,
                    "headers": ["Name", "Value"],
                })

            else:
                # generic label
                widgets.append({
                    "identifier": f"{tid}_lbl",
                    "type": "label",
                    "text": f"{tid}: {round(self._rand.uniform(0,100),1)}",
                })

            tiles_out.append({
                "identifier": tid,
                "widgets": widgets,
            })

        return {"tiles": tiles_out}


# Provide a Qt-friendly adapter when PyQt6 is available.
try:
    from PyQt6.QtCore import QObject, pyqtSignal, QTimer  # type: ignore

    class QtEmulator(QObject):
        """Qt adapter that emits `update` signals on the Qt event loop.

        This avoids threading issues in Qt UIs by using a QTimer to generate
        updates on the GUI thread. If you prefer a background thread, use
        DataEmulator directly and ensure you marshal updates to the GUI thread
        (for example, via QObject signals).
        """

        update = pyqtSignal(dict)

        def __init__(self, interval_ms: int = 500, seed: Optional[int] = None,
                     tile_ids: Optional[List[str]] = None):
            super().__init__()
            self._emulator = DataEmulator(update_interval=interval_ms / 1000.0,
                                          seed=seed, tile_ids=tile_ids)
            self._timer = QTimer(self)
            self._timer.setInterval(max(1, int(interval_ms)))
            self._timer.timeout.connect(self._on_timeout)

        def _on_timeout(self) -> None:
            # generate a fresh payload via the DataEmulator's generator
            payload = self._emulator.generate_update()
            self.update.emit(payload)

        def start(self) -> None:
            self._timer.start()

        def stop(self) -> None:
            self._timer.stop()

except Exception:  # PyQt6 not available
    QtEmulator = None  # type: ignore


if __name__ == "__main__":
    # quick demo of the non-Qt DataEmulator printing a couple updates
    def _print_cb(payload: Dict[str, Any]) -> None:
        print("Got payload with %d tiles" % len(payload.get("tiles", [])))

    em = DataEmulator(update_interval=0.6, seed=42)
    em.register_callback(_print_cb)
    em.start()

    try:
        # run for a few seconds
        time.sleep(3.5)
    finally:
        em.stop()
