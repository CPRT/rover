import serial
from datetime import datetime

ser = serial.Serial(
    "/dev/ttyACM0", 2000000
)  # REPLACE COM4 WITH /dev/ttyUSB0 OR /dev/ttyACM0

last_time = None
filename = datetime.now().strftime("accel_log_%Y%m%d_%H%M%S.csv")
f = open(filename, "w")
f.write("time_us,x,y,z\n")
print(f"Logging to file: {filename}")

while True:
    line = ser.readline().decode("utf-8", errors="ignore").strip()
    if not line:
        continue

    # Detect ESP32 reset: usually time_us goes backwards
    time_us_str = line.split(",")[0]
    try:
        time_us = int(time_us_str)
    except:
        continue

    if last_time is not None and time_us < last_time:
        # ESP32 reset detected → start new CSV
        f.close()
        filename = datetime.now().strftime("accel_log_%Y%m%d_%H%M%S.csv")
        f = open(filename, "w")
        f.write("time_us,x,y,z\n")
        print(f"ESP32 reset detected. Logging to new file: {filename}")

    last_time = time_us
    print(line)  # live debug
    f.write(line + "\n")
