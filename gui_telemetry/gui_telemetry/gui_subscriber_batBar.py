# gui_telemetry/gui_subscriber.py
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, queue, threading, time
from enum import Enum
from typing import Dict, Any, Tuple, List


BACKGROUND = "black"
WINDOW_SIZE = "1200x700"

# --- Define Mode Enum and color mapping ---
class Mode(Enum):
    ESTOP = "Estop"
    AUTO = "Auto"
    MANUAL = "Manual"


def color_bool(v: bool) -> str:
    return "green" if v else "red"


def color_mode(v):
    """Handle both Enum and string modes for safety."""
    if isinstance(v, Mode):
        return {Mode.ESTOP: "red", Mode.AUTO: "blue", Mode.MANUAL: "yellow"}[v]
    elif isinstance(v, str):
        return {"Estop": "red", "Auto": "blue", "Manual": "yellow"}.get(v, "grey")
    return "grey"


def color_batt(v: float) -> str:
    if v >= 11.5:
        return "green"
    if v >= 11.0:
        return "yellow"
    return "red"


# --- GUI Widget ---
class LED(tk.Frame):
    def __init__(self, master, label, size=16):
        bg = master.cget("bg") if hasattr(master, "cget") else BACKGROUND
        super().__init__(master, bg=bg)
        m = 2
        cs = size + 2 * m

        # circular LED indicator
        self.cv = tk.Canvas(self, width=cs, height=cs, highlightthickness=0, bd=0, bg=bg)
        self.cv.grid(row=0, column=0, padx=4, pady=2, sticky="w")
        self.oval = self.cv.create_oval(m, m, m + size, m + size, fill="grey", outline="")

        # label next to LED
        self.var = tk.StringVar(value=label)
        tk.Label(self, textvariable=self.var, bg=bg, fg="white", anchor="w").grid(row=0, column=1, sticky="w")

    def set_color(self, color: str):
        self.cv.itemconfig(self.oval, fill=color)

    def set_label(self, text: str):
        self.var.set(text)

class BatteryBar(tk.Frame):
    def __init__(self, master, label="Battery", min_v=11.0, max_v=12.6):
        super().__init__(master, bg=master.cget("bg"))
        self.min_v = min_v
        self.max_v = max_v

        # Label on left
        self.label_var = tk.StringVar(value=f"{label}: 0.00 V")
        tk.Label(self, textvariable=self.label_var, fg="white", bg=master.cget("bg")).pack(anchor="w")

        # Progress bar (style)
        self.style = ttk.Style()
        self.style.theme_use("default")
        self.style.configure("Battery.Horizontal.TProgressbar",
                             troughcolor="gray25", bordercolor="gray30",
                             background="green", lightcolor="green", darkcolor="green")

        self.bar = ttk.Progressbar(self, style="Battery.Horizontal.TProgressbar",
                                   orient="horizontal", length=180, mode="determinate", maximum=100)
        self.bar.pack(fill="x", padx=4, pady=4)

    def update_value(self, voltage):
        # Clamp voltage to 0–100% range
        pct = max(0, min(100, (voltage - self.min_v) / (self.max_v - self.min_v) * 100))
        self.bar["value"] = pct
        self.label_var.set(f"Battery: {voltage:.2f} V")

        # Color logic
        if voltage >= 11.5:
            color = "green"
        elif voltage >= 11.0:
            color = "yellow"
        else:
            color = "red"
        self.style.configure("Battery.Horizontal.TProgressbar", background=color, lightcolor=color, darkcolor=color)


class LFrame(tk.LabelFrame):
    def __init__(self, master, title):
        super().__init__(master, text=title, padx=8, pady=8, bg=master.cget("bg"), fg="white")
        self.option_add("*Label.foreground", "white")


def gen_repeated(prefix: str, n: int, section: str, subsection: str):
    d = {}
    for i in range(1, n + 1):
        k = f"{prefix}_{i}"
        d[k] = {
            "label": f"{prefix.capitalize()} {i}",
            "default": False,
            "widget": "led",
            "color": color_bool,
            "section": section,
            "subsection": subsection,
        }
    return d


SECTIONS: List[Tuple[str, str]] = [
    ("sensors", "Sensor Status"),
    ("nodes", "Node Status"),
]

# Subgroups for each section
SUBSECTIONS = {
    "sensors": ["Cameras", "Lidars", "Other"],
    "nodes": ["Minion Nodes"]
}

# Master sensor spec
SENSORS: Dict[str, Dict[str, Any]] = {
    # Header items (right, thin bar)
    "boat_hb": {"label": "Boat HB", "default": False, "widget": "led", "color": color_bool,
                "section": "header", "subsection": "Heartbeats"},
    "drone_hb": {"label": "Drone HB", "default": False, "widget": "led", "color": color_bool,
                 "section": "header", "subsection": "Heartbeats"},
    "mode": {"label": "Mode", "default": Mode.ESTOP, "widget": "led", "color": color_mode,
             "section": "header", "subsection": "Heartbeats"},
    "battery": {"label":"Battery (V)","default":11.3,"widget":"battery",
                "section":"header","subsection":"Battery"},


    # Sensors
    "gps_lock": {"label": "GPS Lock", "default": False, "widget": "led", "color": color_bool,
                 "section": "sensors", "subsection": "Other"},

    # Nodes
    "ros_control": {"label": "Control", "default": False, "widget": "led", "color": color_bool,
                    "section": "nodes", "subsection": "Minion Nodes"},
    "ros_path": {"label": "Path Planning", "default": False, "widget": "led", "color": color_bool,
                 "section": "nodes", "subsection": "Minion Nodes"},
    "ros_yolo": {"label": "YOLO", "default": False, "widget": "led", "color": color_bool,
                 "section": "nodes", "subsection": "Minion Nodes"},
    "ros_gbcache": {"label": "GBCACHE", "default": False, "widget": "led", "color": color_bool,
                    "section": "nodes", "subsection": "Minion Nodes"},
    "ros_map": {"label": "Mapper", "default": False, "widget": "led", "color": color_bool,
                "section": "nodes", "subsection": "Minion Nodes"},
    "ros_ROI": {"label": "ROIs", "default": False, "widget": "led", "color": color_bool,
                "section": "nodes", "subsection": "Minion Nodes"},
}

# Add camera and lidar indicators
SENSORS.update(gen_repeated("camera", 6, "sensors", "Cameras"))
SENSORS.update(gen_repeated("lidar", 6, "sensors", "Lidars"))


# --- ROS Node ---
class TelemetrySubscriber(Node):
    def __init__(self, q):
        super().__init__('telemetry_subscriber')
        self.q = q
        self.subscription = self.create_subscription(String, 'telemetry', self.listener_callback, 10)

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.q.put_nowait(data)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse telemetry: {e}")


# --- GUI ---
class App(tk.Tk):
    POLL_MS = 50
    NUM_SENSOR_SUBGROUP_COLS = 3
    INDICATOR_COLS_PER_SUBGROUP = 2

    def __init__(self, q):
        super().__init__()
        self.q = q
        self.geometry(WINDOW_SIZE)
        self.configure(bg=BACKGROUND)
        self.state = {k: spec["default"] for k, spec in SENSORS.items()}

        root = tk.Frame(self, bg=BACKGROUND)
        root.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        root.grid_columnconfigure(0, weight=1)
        root.grid_columnconfigure(1, weight=2)
        root.grid_columnconfigure(2, weight=1)
        root.grid_rowconfigure(0, weight=1)
        root.grid_rowconfigure(1, weight=1)
        root.grid_rowconfigure(2, weight=1)
        
        # --- Left: Sensors (row 0) + Nodes (row 1)
        self.sensor_section = tk.Frame(root, bg=BACKGROUND)
        self.sensor_section.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        tk.Label(self.sensor_section, text="Sensor Status", bg=BACKGROUND, fg="white",
                 font=("Arial", 20)).pack(anchor="w", pady=(0, 8))

        self.node_section = tk.Frame(root, bg=BACKGROUND)
        self.node_section.grid(row=1, column=0, sticky="nsew", padx=(0, 10))
        tk.Label(self.node_section, text="Node Status", bg=BACKGROUND, fg="white",
                 font=("Arial", 20)).pack(anchor="w", pady=(0, 8))

        self.time_section = tk.Frame(root, bg=BACKGROUND)
        self.time_section.grid(row=2, column=0, sticky="nsew", padx=(0,10))
        tk.Label(self.time_section, text="Time", bg=BACKGROUND, fg="white",
                 font=("Arial", 20)).pack(anchor="w",pady=(0,8))

        # --- Right: header + tasks
        right = tk.Frame(root, bg=BACKGROUND)
        right.grid(row=0, column=1, rowspan=2, sticky="nsew")
        right.grid_rowconfigure(1, weight=1)

        # Header (fixed height)
        header = tk.Frame(right, bg="grey20", height=64)
        header.grid(row=0, column=0, sticky="ew", padx=6, pady=(0, 6))
        header.grid_propagate(False)
        header_inner = tk.Frame(header, bg="grey20")
        header_inner.pack(fill=tk.X, expand=True, padx=8, pady=8)

        self.header_frames = {
            "Heartbeats": LFrame(header_inner, "Heartbeats"),
            "Battery": LFrame(header_inner, "Battery"),
        }
        self.header_frames["Heartbeats"].grid(row=0, column=0, sticky="w", padx=(0, 8))
        self.header_frames["Battery"].grid(row=0, column=1, sticky="e")

        # --- Tasks
        tasks = tk.Frame(right, bg=BACKGROUND)
        tasks.grid(row=1, column=0, sticky="nsew")
        tk.Label(tasks, text="Tasks", bg=BACKGROUND, fg="white", font=("Arial", 18)).pack(anchor="w", padx=6, pady=(6, 2))
        lb = tk.Listbox(tasks, bg="black", fg="white", bd=0, highlightthickness=0)
        lb.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)
        for t in ["Inspect Camera 1", "Restart perception", "Check GPS"]:
            lb.insert(tk.END, t)

        # --- Build header LEDs
        self.widgets: Dict[str, LED] = {}
        for key in ("boat_hb", "drone_hb", "mode"):
            w = LED(self.header_frames["Heartbeats"], label=SENSORS[key]["label"], size=16)
            w.grid(sticky="w", padx=4, pady=2)
            self.widgets[key] = w
        w = LED(self.header_frames["Battery"], label=SENSORS["battery"]["label"], size=16)
        self.widgets["battery"] = BatteryBar(self.header_frames["Battery"], label="Battery")
        self.widgets["battery"].grid(sticky="ew", padx=4, pady=2)


        # --- Build sections
        self._build_section_with_grid(
            parent=self.sensor_section,
            section_key="sensors",
            subgroup_cols=self.NUM_SENSOR_SUBGROUP_COLS,
            indicator_cols=self.INDICATOR_COLS_PER_SUBGROUP
        )
        self._build_section_with_grid(
            parent=self.node_section,
            section_key="nodes",
            subgroup_cols=2,
            indicator_cols=2
        )

        self._render_all()
        self.after(self.POLL_MS, self._poll)
        self.protocol("WM_DELETE_WINDOW", self.destroy)

    def _build_section_with_grid(self, parent: tk.Frame, section_key: str, subgroup_cols: int, indicator_cols: int):
        inner = tk.Frame(parent, bg=BACKGROUND)
        inner.pack(fill=tk.BOTH, expand=True)

        for c in range(subgroup_cols):
            inner.grid_columnconfigure(c, weight=1, uniform=f"{section_key}_cols")

        subframes: Dict[str, LFrame] = {}
        r = c = 0
        for subname in SUBSECTIONS[section_key]:
            lf = LFrame(inner, subname)
            lf.grid(row=r, column=c, sticky="nsew", padx=8, pady=8)
            subframes[subname] = lf
            c = (c + 1) % subgroup_cols
            if c == 0:
                r += 1

        cursors: Dict[str, Tuple[int, int]] = {s: (0, 0) for s in SUBSECTIONS[section_key]}

        def alloc(sub: str) -> Tuple[int, int]:
            r, c = cursors[sub]
            c_new = (c + 1) % indicator_cols
            r_new = r if c_new != 0 else r + 1
            cursors[sub] = (r_new, c_new)
            return r, c

        for key, spec in SENSORS.items():
            if spec["section"] != section_key or spec["widget"] != "led":
                continue
            frame = subframes[spec["subsection"]]
            r, c = alloc(spec["subsection"])
            w = LED(frame, label=spec["label"], size=16)
            w.grid(row=r, column=c, padx=6, pady=3, sticky="w")
            frame.grid_columnconfigure(c, weight=0)
            self.widgets[key] = w

    def _poll(self):
        try:
            data = self.q.get_nowait()
            if data:
                for k, v in data.items():
                    if k in self.state:
                        self.state[k] = v
                self._render_all()
        except queue.Empty:
            pass
        self.after(self.POLL_MS, self._poll)

    def _render_all(self):
        mode = self.state.get("mode", Mode.ESTOP)
        self.configure(bg=color_mode(mode))

        for key, spec in SENSORS.items():
            if spec["widget"] != "led":
                continue
            val = self.state.get(key, spec["default"])
            color = spec["color"](val)
            self.widgets[key].set_color(color)

        bv = self.state.get("battery", None)
        if isinstance(bv, (int, float)):
            self.widgets["battery"].update_value(bv)


def main():
    q = queue.Queue()
    rclpy.init()

    node = TelemetrySubscriber(q)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = App(q)
    app.mainloop()

    rclpy.shutdown()
    spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
