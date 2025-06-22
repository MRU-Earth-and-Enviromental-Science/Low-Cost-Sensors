# 🌿 Air Quality Sensor

An air quality monitoring system designed for DJI Matrice drones, capable of measuring and logging key environmental parameters: CO₂, CO, NOₓ, PM2.5, VOCs, temperature, and humidity. 

This system integrates:
- An ESP32 for real-time sensor data acquisition
- A Raspberry Pi running ROS Noetic and DJI Onboard SDK
- ESP-NOW for wireless transmission to a ground station
-  A web dashboard for live monitoring and CSV export

---

## 🚀 Features
- Real-time measurements of:
  - CO<sub>2</sub> in ppm
  - CO (Carbon Monoxide) in ppm
  - NO<sub>x</sub> in ppm
  - PM2.5 particulate matter in μg/m<sup>3</sup>
  - VOCs (Volatile Organic Compounds)
  - Temperature (°C) and Humidity (%)
- Web-based dashboard for live data visualization.
- PlatformIO-based development environment.
- ROS Noetic for drone-Pi-ESP communication (via UART)
- Custom PCB + 3D-printed casing for DJI Matrice 210V2 (compatible with 350RTK)

---

## 🛠️ Getting Started (Software)

### 1. Prerequisites
- Familiarity with terminal and basic Linux commands
- Recommended: Linux host (Windows/Mac supported but requires extra configuration)
- **Required Tools**
  - C++ Complier
  - [VSCode](https://code.visualstudio.com/)
  - [Platform IO Extension](https://platformio.org/install/ide?install=vscode) for VSCode
  - (Optional) Install [PlatformIO Core CLI](https://docs.platformio.org/en/latest/core/quickstart.html) for terminal-only workflows
  - [Optional](https://www.docker.com/get-started/) (for running ROS & DJI SDK in containers, currently not required, for future updates)
    
### 2. Clone the Repository
```bash
git clone https://github.com/yourusername/Low-Costs-Sensors-Gas.git
cd Low-Costs-Sensors-Gas
code . # Open in VSCode
```
### 3. Upload Code to ESP32

- Connect your **ESP32 dev board** via USB.  
- Open the Drone_System Directory on **Visual Studio Code**.
```bash
cd path-to-directory/Low-Costs-Sensors-Gas/Drone_System
``` 
- Use PlatformIO to build and upload:  
  - Click the right-facing arrow (➤) at the bottom of VSCode, or  
  - Use the command palette: `PlatformIO: Upload`  

> PlatformIO will automatically detect your environment and upload the firmware to the board.

### 4. Raspberry Pi Set-Up
- Running any modern Linux Distro (This was developed on Ubuntu 24.04)
- Setup Docker Daemon (if wanting to use DJI OSDK)
```bash
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo docker run hello-world

```
- clone this repo
- compile main.cpp
- make it run on boot
---

## 🧰 Hardware Used

- **ESP32 Dev Module (WROVER-E based)**
- **Raspberry Pi 5**
- **K30 CO₂ Sensor**
- **MQ Series Gas Sensors** (e.g., MQ-7 for CO, MQ-135 for NOₓ/CH₄)
- **Plantower PMS7003** (PM2.5 Sensor)
- **SGP30** (VOC Sensor)
- **DHT11 / DHT22** (Temperature & Humidity)
- **Custom PCB** Ordered from JLC PCB

## Casing
- STEP and STL Files avaliable

---

## 📁 Project Structure

```
/GroundStationReader/               # Receives ESP-NOW data and handles web upload
  ├── main.py                       # Python script for ESP-NOW data reception and API push
  └── web_app/                      # Web dashboard (HTML/CSS/JS)

/PlatformIO/                        # ESP32 firmware (sensor reading + ESP-NOW transmission)
  ├── src/                          # Source files for sensor logic and communication
  ├── include/                      # Header files for modular components
  └── platformio.ini                # PlatformIO config for board, libraries, etc.

/RaspDocker/                        # Dockerized ROS Noetic + DJI OSDK (runs on Raspberry Pi)
  ├── OnboardSDK/                   # DJI Onboard SDK (git submodule)
  ├── catkin_ws/
  │   └── src/drone_data/           # Custom ROS package
  │       ├── src/                  # ROS node source code
  │       ├── CMakeLists.txt
  │       └── package.xml
  ├── Dockerfile                    # Builds ROS container with DJI SDK
  └── run.sh                        # Launches ROS node and UART communication

README.md                           # Project documentation
LICENSE                             # MIT License
```
---

## 📄 License
This project is licensed under the **MIT License**.

