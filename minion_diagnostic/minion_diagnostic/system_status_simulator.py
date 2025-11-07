import random
from datetime import datetime, timedelta
from typing import Dict


class FakeSystemStatus:
    def __init__(self, seed: int | None = None):
        if seed is not None:
            random.seed(seed)

        # Start in a realistic range (not extreme random every run)
        self.ambient_temp = random.uniform(20.0, 28.0)
        self.cpu_usage = random.uniform(2.0, 30.0)
        self.gpu_usage = random.uniform(0.0, 30.0)
        # Temperature depends on ambient + usage
        self.temperature = self.ambient_temp + 0.3 * self.cpu_usage + 0.2 * self.gpu_usage + random.uniform(-1, 1)
        self.battery_percentage = random.uniform(30.0, 100.0)
        # More likely to be plugged in
        self.battery_plugged = random.choice([False, False, True])
        self.linux_version = "Windows 11 (Mock)"  # Or "Ubuntu 22.04"
        self.current_time = datetime.now()

        self.ram_usage = random.uniform(10.0, 70.0)
        # storage_used is percent used of disk
        self.storage_used = random.uniform(5.0, 70.0)

        # internal small state used to create bursty behaviour
        self._cpu_trend = random.uniform(-0.5, 0.5)
        self._gpu_trend = random.uniform(-0.5, 0.5)

    def _clamp(self, v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    def step(self, seconds: float = 1.0) -> None:
        """Advance the simulation by a number of seconds.

        The model is intentionally lightweight: random-walk for CPU/GPU with
        occasional spikes, temperature coupled to usage, battery charging or
        draining depending on plugged state, and slow RAM/storage drift.
        """

        # Random-walk CPU/GPU with a slight inertia (trend)
        cpu_noise = random.gauss(0, 3.0)
        gpu_noise = random.gauss(0, 2.0)

        # Occasionally create short spikes (like a task starting)
        if random.random() < 0.02:
            cpu_noise += random.uniform(15, 60) * random.choice([1, -1])
        if random.random() < 0.015:
            gpu_noise += random.uniform(10, 50) * random.choice([1, -1])

        # Apply trend and noise scaled by seconds
        self._cpu_trend = 0.9 * self._cpu_trend + 0.1 * random.uniform(-1, 1)
        self._gpu_trend = 0.9 * self._gpu_trend + 0.1 * random.uniform(-1, 1)

        self.cpu_usage = self._clamp(self.cpu_usage + (self._cpu_trend + cpu_noise) * (seconds / 1.0), 0.0, 100.0)
        self.gpu_usage = self._clamp(self.gpu_usage + (self._gpu_trend + gpu_noise) * (seconds / 1.0), 0.0, 100.0)

        # RAM usage loosely follows CPU usage (e.g., apps starting)
        ram_noise = random.gauss(0, 1.5)
        self.ram_usage = self._clamp(self.ram_usage + (0.02 * (self.cpu_usage - self.ram_usage) + ram_noise) * (seconds / 1.0), 0.0, 100.0)

        # Storage changes very slowly (writes / cleans)
        if random.random() < 0.05:
            # small write
            self.storage_used = self._clamp(self.storage_used + random.uniform(0.01, 0.2) * seconds, 0.0, 100.0)
        elif random.random() < 0.01:
            # cleanup
            self.storage_used = self._clamp(self.storage_used - random.uniform(0.01, 0.5) * seconds, 0.0, 100.0)

        # Temperature reacts to CPU/GPU usage and slowly relaxes toward ambient when idle
        usage_effect = 0.28 * self.cpu_usage + 0.18 * self.gpu_usage
        temp_noise = random.gauss(0, 0.3)
        # Smooth the temperature change so it doesn't jump wildly
        desired_temp = self.ambient_temp + usage_effect
        self.temperature = self._clamp(self.temperature + (desired_temp - self.temperature) * 0.1 + temp_noise * (seconds / 1.0), -50.0, 150.0)

        # Battery charging/draining
        if self.battery_plugged:
            # charge slowly, slower when near 100%
            charge_rate = 0.15  # percent per second (fast charger simulation)
            # reduce charge rate as battery approaches full
            charge_rate *= (1.0 - (self.battery_percentage / 100.0))
            # small jitter
            self.battery_percentage = self._clamp(self.battery_percentage + charge_rate * seconds + random.uniform(-0.02, 0.05), 0.0, 100.0)
        else:
            # drain depends on CPU/GPU usage
            base_drain = 0.01  # percent per second idle
            usage_drain = (0.0015 * self.cpu_usage + 0.0008 * self.gpu_usage)  # %/s
            drain = (base_drain + usage_drain) * seconds
            # occasional heavier drain (e.g., background work)
            if random.random() < 0.01:
                drain += random.uniform(0.05, 0.5)
            self.battery_percentage = self._clamp(self.battery_percentage - drain, 0.0, 100.0)

        # Advance the clock
        self.current_time = self.current_time + timedelta(seconds=seconds) if hasattr(self, 'current_time') else datetime.now()

    def snapshot(self, seconds: float = 1.0) -> Dict[str, str]:
        """Advance by `seconds` and return a formatted snapshot dict."""
        # Update internal state
        self.step(seconds)

        # Format for display
        return {
            "CPU Usage": f"{self.cpu_usage:.1f}%",
            "GPU Usage": f"{self.gpu_usage:.1f}%",
            "Temperature": f"{self.temperature:.1f} °C",
            "Battery": f"{self.battery_percentage:.1f}%",
            "Charging": "Yes" if self.battery_plugged else "No",
            "OS": self.linux_version,
            "Time": self.current_time.strftime('%Y-%m-%d %H:%M:%S') if hasattr(self.current_time, 'strftime') else str(self.current_time),
            "RAM Usage": f"{self.ram_usage:.1f}%",
            "Storage": f"{self.storage_used:.1f}%"
        }