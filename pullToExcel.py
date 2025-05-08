import requests
import pandas as pd
from datetime import datetime
import time


data_log = []

# Optional: show headers for the first row
print("Timestamp | Temperature (°C) | Humidity (%) | CH4 (ppm)")

while True:
    try:
        # Send request to ESP32
        response = requests.get("http://192.168.4.1/data", timeout=3)

        if response.status_code == 200:
            data = response.json()
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            entry = {
                "Timestamp": timestamp,
                "Temperature (°C)": data["temperature"],
                "Humidity (%)": data["humidity"],
                "CH4 (ppm)": data["ch4_ppm"],
                "CO2 (ppm)": data["co2_ppm"]
            }

            # Append new row
            data_log.append(entry)

            # Save to Excel every time
            df = pd.DataFrame(data_log)
            df.to_csv("esp32_sensor_log.csv", index=False)

            print(
                f"{timestamp} | {data['temperature']} | {data['humidity']} | {data['ch4_ppm']} | {data['co2_ppm']}")

        time.sleep(1)  # Log every second

    except Exception as e:
        print("Error:", e)
        time.sleep(5)
