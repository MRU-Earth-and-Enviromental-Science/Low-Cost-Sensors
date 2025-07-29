# 🌿 Air Quality Sensor

An air quality monitoring system designed for DJI Matrice drones, capable of measuring and logging key environmental parameters: CO<sub>2</sub>, CO, CH<sub>4</sub>, NO<sub>x</sub>, PM<sub>2.5</sub>, VOCs, temperature, and humidity. 

This system integrates:
- An ESP32 for real-time sensor data acquisition
- ESP-NOW for wireless transmission to a ground station
- A web dashboard for live monitoring and CSV export
- Raspberry Pi running ROS Noetic and DJI Onboard SDK

---
![License](https://img.shields.io/badge/License-MIT-000000?style=for-the-badge&logo=openaccess&logoColor=white)
![DJI](https://img.shields.io/badge/DJI-000000?style=for-the-badge&logo=dji&logoColor=white)

![ROS Noetic](https://img.shields.io/badge/ROS%20Noetic-000000?style=for-the-badge&logo=ROS&logoColor=white)
![Linux](https://img.shields.io/badge/Linux-000000?style=for-the-badge&logo=linux&logoColor=white)
![PlatformIO](https://img.shields.io/badge/PlatformIO-000000?style=for-the-badge&logo=platformio&logoColor=white)
![ESP32](https://img.shields.io/badge/ESP32-000000?style=for-the-badge&logo=espressif&logoColor=white)
![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-000000?style=for-the-badge&logo=raspberrypi&logoColor=white)
![C++](https://img.shields.io/badge/C++-000000?style=for-the-badge&logo=cplusplus&logoColor=white)
![Python](https://img.shields.io/badge/Python-000000?style=for-the-badge&logo=python&logoColor=white)
![Bash](https://img.shields.io/badge/Bash-000000?style=for-the-badge&logo=gnu-bash&logoColor=white)
![TypeScript](https://img.shields.io/badge/TypeScript-000000?style=for-the-badge&logo=typescript&logoColor=white)


## 🚀 Features
- Real-time measurements of:
  - CO<sub>2</sub> in ppm
  - CO (Carbon Monoxide) in ppm
  - CH<sub>4</sub> in ppm
  - NO<sub>x</sub> in ppm
  - PM2.5 particulate matter in μg/m<sup>3</sup>
  - VOCs (Volatile Organic Compounds)
  - Temperature (°C) and Humidity (%)
- Web-based dashboard for live data visualization.
  - Built using Electron, Typescript, Next.js, Python and [v0.dev](https://v0.dev)
- PlatformIO-based development environment.
- ROS Noetic for drone-Pi-ESP communication (via UART)
  - Allows for GPS data to be pulled from the drone
- Custom PCB + 3D-printed casing for DJI Matrice 210V2 (Matrice 350RTK Coming Soon)

---

## 🛠️ Getting Started (Software)

### 1. Prerequisites
- Familiarity with Git, terminal and basic UNIX commands
- Recommended: Linux host (Windows/Mac supported but requires extra configuration)
- **Required Tools**
  - C++ Compiler
    - Linux: `sudo apt install build-essential`
    - Mac: Xcode Command Line Tools (`xcode-select --install`)
    - Windows: Install [MinGW](https://www.mingw-w64.org/) or use WSL
  - [VSCode](https://code.visualstudio.com/) (Or IDE of your choice)
  - [Platform IO Extension](https://platformio.org/install/ide?install=vscode) for VSCode
  - [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) (This software was built in 2025 on Ubuntu 20.04)
  - [Optional but Recommended] [PlatformIO Core CLI](https://docs.platformio.org/en/latest/core/quickstart.html) for terminal workflows
    
### 2. Clone the Repository on the Machine Used to Deploy to ESP32
```bash
cd ~
git clone https://github.com/MRU-Earth-and-Enviromental-Science/Low-Cost-Sensors.git
cd Low-Cost-Sensors
code . # Open in VSCode (or editor of your choice)
```
### 3. Upload Code to ESP32 on the Drone

- Connect your **ESP32 dev board** via USB.
- Open the Drone_System Directory on **Visual Studio Code**.
```bash
cd ~/Low-Cost-Sensors/drone_esp32
``` 
- Use PlatformIO to build and upload:  
  - Click the right-facing arrow (➤) at the bottom of VSCode, or  
```shell
# If using PIO command line
pio run --target upload
```

> PlatformIO will automatically detect your environment and upload the firmware to the board.

### 4. Configure the ESP32 on Ground Station
```bash
cd ~/Low-Cost-Sensors/ground_station/ground_esp32
```
- Use PlatformIO to build and upload the code to the ESP32. (same as above)

### 5. Raspberry Pi Set-Up
- Running any Linux Distro between 16.04 and 20.04 (This was developed on Ubuntu 20.04 Server, Raspberry Pi 3)
- Requires ROS Noetic (Base) to be installed
- Requires a C++ compiler
- Requires [CMake](https://cmake.org/download/) > 3.0
- Clone this repo on the Pi
```bash
cd ~
git clone https://github.com/MRU-Earth-and-Enviromental-Science/Low-Cost-Sensors.git
```
- Clone the DJI OSDK and DJI OSDK ROS Packages
```bash
cd ~
git clone https://github.com/dji-sdk/Onboard-SDK.git

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
git clone https://github.com/dji-sdk/Onboard-SDK-ROS/tree/master/src/dji_osdk_ros
```
Run the install script to setup the rest of the code
```bash
cd ~/Low-Cost-Sensors
chmod +x install_sdk.sh
./install_sdk.sh
```

### 6. Setting Up and Running the Dashboard
- Install the Dashboard from the 'Releases Section' of the GitHub repo linked below for your system.
  - For **Windows**: Download the `.exe` file.
  - For **Mac**: Download the `.dmg` file.
  - For **Linux**: Download the `.AppImage` file and make it executable:
- Alternatively, you can clone the sensor-dashboard submodule and build from source (must have npm installed):
```bash
# Clone the sensor-dashboard submodule
cd ~/Low-Cost-Sensors
git submodule update --init --recursive

cd ground_station/sensor-dashboard

npm install --legacy-peer-deps
npm run dev

# in a new terminal, run the following command to start the dashboard
npm run electron-dev

# to run the backend
python3 -m venv venv
source venv/bin/activate # This is a UNIX command for windows run: venv\Scripts\activate
pip3 install -r requirements.txt

python3 serial_backend.py # Run this in the venv
```
---
## 🧰 Hardware Used
- **ESP32 Dev Module (WROVER-E based)**
- **Raspberry Pi 3**
- **K30 CO₂ Sensor**
- **MQ Series Gas Sensors** (e.g., MQ-7 for CO, MQ-135 for NOₓ, MQ-4 for CH₄)
- **Plantower PMS7003** (PM2.5 Sensor)
- **SGP30** (VOC Sensor)
- **DHT11** (Temperature & Humidity)
- **UART to USB Cable** (Drone to Pi connection)
- **Custom PCB** Ordered from JLC PCB (Found in Repo)
- **Bill of Materials Found in Repo**

## Casing
- STEP and STL Files available to mount to Matrice210
---

## 📄 License
This project is licensed under the **MIT License**.

## 🛠️ Developed By

[Mount Royal University, Faculty of Science and Technology, Department of Earth and Environmental Science](https://www.mtroyal.ca/ProgramsCourses/FacultiesSchoolsCentres/ScienceTechnology/Departments/EarthEnvironmentalSciences/index.htm)

[Gwen O'Sullivan](https://www.mtroyal.ca/ProgramsCourses/FacultiesSchoolsCentres/ScienceTechnology/office-of-the-dean/index.htm) — Vice Dean of Science and Technology @ MRU \
[Shivam Walia](https://www.linkedin.com/in/shivam-walia1/) — Mechatronics Engineering @ UWaterloo

## ❓ Questions
[shivamwalia2006 [at] gmail [dot] com](mailto:shivamwalia2006@gmail.com) \
[gosullivan [at] mtroyal [dot] ca](mailto:gosullivan@mtroyal.ca)
