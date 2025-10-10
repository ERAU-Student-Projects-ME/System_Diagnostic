import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, random, time
from enum import Enum


# ---------- Mode Enum ----------
class Mode(Enum):
    ESTOP = "Estop"
    AUTO = "Auto"
    MANUAL = "Manual"


# ---------- Telemetry Publisher ----------
class TelemetryPublisher(Node):
    def __init__(self):
        super().__init__('publisher')

        # Create publisher for topic "telemetry"
        self.pub = self.create_publisher(String, 'telemetry', 10)

        # Publish rate: 10 Hz
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_packet)

        # State variables
        self.modes = [Mode.ESTOP, Mode.AUTO, Mode.MANUAL]
        self.i = 0
        self.t0 = time.monotonic()
        self.tick = 0

        self.get_logger().info("TelemetryPublisher started, publishing on /telemetry")

    def publish_packet(self):
        self.tick += 1
        now = time.monotonic()

        # Every ~2 s, cycle to next mode
        if now - self.t0 > 2.0:
            self.i = (self.i + 1) % len(self.modes)
            self.t0 = now

        # Build telemetry packet (same structure as old threaded version)
        pkt = {
            "mode": self.modes[self.i].value,
            "boat_hb": True,
            "drone_hb": True,
            "battery": 11.1 + 0.9 * random.random(),
            "gps_lock": (self.tick // 10) % 3 != 0,
            "ros_control": (self.tick % 50) != 0,
            "ros_path": (self.tick % 7) != 0,
            "ros_yolo": True,
            "ros_gbcache": (self.tick % 50) != 0,
            "ros_map": (self.tick % 25) != 0,
            "ros_ROI": (self.tick % 12) != 0,
            "time": self.t0,
             }

        # Add camera and lidar states
        for i in range(1, 7):
            pkt[f"camera_{i}"] = random.random() > 0.1
            pkt[f"lidar_{i}"] = random.random() > 0.2

        # Convert to JSON string and publish
        msg = String()
        msg.data = json.dumps(pkt)
        self.pub.publish(msg)
        # Optional debug log
        # self.get_logger().info(f"Published: {msg.data}")


# ---------- Main Entry ----------
def main(args=None):
    rclpy.init(args=args)
    node = TelemetryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
