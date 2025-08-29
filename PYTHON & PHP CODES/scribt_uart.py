import serial
import time
import os
from datetime import datetime

# Serial port
ser = serial.Serial('COM6', 9600, timeout=1)

# Output file paths
base_path = "C:/xampp/htdocs/xampp/"
temp_tmp = os.path.join(base_path, "temp_tmp.txt")
temp_final = os.path.join(base_path, "temp.txt")

press_tmp = os.path.join(base_path, "press_tmp.txt")
press_final = os.path.join(base_path, "press.txt")

hum_tmp = os.path.join(base_path, "hum_tmp.txt")
hum_final = os.path.join(base_path, "hum.txt")

# clear old values on startup
for tmp in [temp_final, press_final, hum_final]:
    open(tmp, "w").close()

while True:
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()

        if not line:
            continue

        print(f"[{datetime.now()}] Received: {line}")

        if "Temp:" in line:
            with open(temp_tmp, "w") as f:
                f.write(line)
            os.replace(temp_tmp, temp_final)

        elif "Press:" in line:
            with open(press_tmp, "w") as f:
                f.write(line)
            os.replace(press_tmp, press_final)

        elif "Hum:" in line:
            with open(hum_tmp, "w") as f:
                f.write(line)
            os.replace(hum_tmp, hum_final)

    except Exception as e:
        print("ERROR:", e)
