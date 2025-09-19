import rclpy
from rclpy.node import Node
import psutil
import platform
import subprocess
import math
from datetime import datetime
import glob
import os

from minion_diagnostic_msg.msg import SystemStatus  # Replace with your package name


class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        self.publisher_ = self.create_publisher(SystemStatus, 'system_status', 10)
        self.timer = self.create_timer(2.0, self.publish_status)

    def get_gpu_usage(self):
        # Try NVIDIA first
        try:
            output = subprocess.check_output(
                "nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits",
                shell=True, stderr=subprocess.DEVNULL
            ).decode().strip()
            return float(output)
        except Exception:
            pass

        # Try AMD next (Linux only)
        try:
            amd_paths = glob.glob("/sys/class/drm/card*/device/gpu_busy_percent")
            if amd_paths:
                with open(amd_paths[0], "r") as f:
                    return float(f.read().strip())
        except Exception:
            pass

        # Intel iGPU (not straightforward)
        # Could add intel_gpu_top parsing in future, but default to NaN
        return math.nan

    def publish_status(self):
        msg = SystemStatus()

        # CPU usage
        msg.cpu_usage = psutil.cpu_percent()

        # GPU usage
        msg.gpu_usage = self.get_gpu_usage()

        # Temperature
        try:
            temps = psutil.sensors_temperatures()
            if "coretemp" in temps:
                msg.temperature = float(temps["coretemp"][0].current)
            elif len(temps) > 0:
                msg.temperature = float(list(temps.values())[0][0].current)
            else:
                msg.temperature = math.nan
        except Exception:
            msg.temperature = math.nan

        # Battery
        try:
            battery = psutil.sensors_battery()
            if battery:
                msg.battery_percentage = battery.percent
                msg.battery_plugged = battery.power_plugged
            else:
                msg.battery_percentage = -1.0
                msg.battery_plugged = False
        except Exception:
            msg.battery_percentage = -1.0
            msg.battery_plugged = False

        # Linux version
        msg.linux_version = f"{platform.system()} {platform.release()}"

        # Current time
        msg.current_time = datetime.now().isoformat()

        # RAM usage
        msg.ram_usage = psutil.virtual_memory().percent

        # Storage usage (root fs)
        msg.storage_used = psutil.disk_usage('/').percent

        # Publish
        self.publisher_.publish(msg)
        # self.get_logger().info(
        #     f"CPU {msg.cpu_usage:.1f}%, GPU {msg.gpu_usage if not math.isnan(msg.gpu_usage) else 'NaN'}%, "
        #     f"Temp {msg.temperature:.1f}°C, RAM {msg.ram_usage:.1f}%, Disk {msg.storage_used:.1f}%"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
