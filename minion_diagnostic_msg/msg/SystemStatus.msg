# SystemStatus.msg
float32 cpu_usage           # % CPU usage
float32 gpu_usage           # % GPU usage (-1 if not available)
float32 temperature         # CPU temperature in °C (NaN if not available)
float32 battery_percentage  # % Battery remaining (-1 if not available)
bool   battery_plugged      # Charging status
string linux_version        # Kernel version / Linux release
string current_time         # Current system time (ISO 8601 format)
float32 ram_usage           # % RAM usage
float32 storage_used        # % total storage used

