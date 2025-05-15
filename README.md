# 🌿 Air Quality Sensor

An ESP32-based air quality monitoring system designed to measure and log environmental parameters such as **CO2, CO, PM2.5, VOCs, temperature, and humidity**. Data is transmitted via Wi-Fi and visualized through a web dashboard. The sensor is designed to be fitted onto a DJI drone.

---

## 🚀 Features
- Real-time measurements of:
  - CO2 concentration
  - CO (Carbon Monoxide)
  - PM2.5 particulate matter
  - VOCs (Volatile Organic Compounds)
  - Temperature and Humidity
- Web-based dashboard for live data visualization.
- PlatformIO-based development environment.
- Expandable for additional sensors and data logging.

---

## 🛠️ Getting Started

### 1. Prerequisites
- **Visual Studio Code (VSCode)**  
  [Download VSCode](https://code.visualstudio.com/)
- **PlatformIO IDE extension for VSCode**  
  [Install PlatformIO Extension](https://platformio.org/install/ide?install=vscode)
- *(Optional)* **PlatformIO Core CLI**  
  For terminal usage: [PlatformIO Core Quickstart](https://docs.platformio.org/en/latest/core/quickstart.html)

### 2. Clone the Repository
```bash
git clone https://github.com/yourusername/air-quality-sensor.git
cd air-quality-sensor
```
## 🚀 3. Open in VSCode & Build
- Open the folder in **Visual Studio Code**.
- PlatformIO will auto-detect the environment.
- Build the project using:
  - `PlatformIO: Build` (checkmark icon in status bar).

## 🚀 4. Upload Code to ESP32
- Connect your **ESP32 dev board** via USB.
- Upload firmware using:
  - `PlatformIO: Upload`.

## 🚀 5. Monitor Serial Output
- Open the **PlatformIO Serial Monitor** to view real-time sensor data:
  - `PlatformIO: Monitor`.

---

## 🧰 Hardware Used
- **ESP32 Dev Module (WROVER-E based)**
- **K30 CO2 Sensor**
- **MQ series sensors (CO, CH4)**
- **Keyestudio PM2.5 Sensor**
- **SGP30 VOC Sensor**
- **DHT11 / DHT22 (Temperature & Humidity)**

---

## 🖥️ Web Dashboard
- Hosted on ESP32’s web server.
- Access via device IP address on the local network.
- Displays live sensor data in an easy-to-read format.

---

## 📁 Project Structure
```
/lib        # Sensor drivers & helper libraries
/src        # Main application code
/include    # Header files
/platformio.ini  # Environment configuration
/README.md  # This file
```
---

## 📄 License
This project is licensed under the **MIT License**.

