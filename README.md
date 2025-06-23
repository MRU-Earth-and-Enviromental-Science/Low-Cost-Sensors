# 🌿 Air Quality Sensor

An air quality monitoring system designed for DJI Matrice drones, capable of measuring and logging key environmental parameters: CO₂, CO, NOₓ, PM2.5, VOCs, temperature, and humidity. 

This system integrates:
- An ESP32 for real-time sensor data acquisition
- ESP-NOW for wireless transmission to a ground station
- A web dashboard for live monitoring and CSV export
- Raspberry Pi running ROS Noetic and DJI Onboard SDK (coming soon)

---
![License](https://img.shields.io/badge/License-MIT-000000?style=for-the-badge&logo=openaccess&logoColor=white)
![PlatformIO](https://img.shields.io/badge/PlatformIO-000000?style=for-the-badge&logo=platformio&logoColor=white)
![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-000000?style=for-the-badge&logo=raspberrypi&logoColor=white)
![DJI](https://img.shields.io/badge/DJI-000000?style=for-the-badge&logo=dji&logoColor=white)
![Linux](https://img.shields.io/badge/Linux-000000?style=for-the-badge&logo=linux&logoColor=white)
![TypeScript](https://img.shields.io/badge/TypeScript-000000?style=for-the-badge&logo=typescript&logoColor=white)

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
- ROS Noetic (coming soon) for drone-Pi-ESP communication (via UART)
- Custom PCB + 3D-printed casing for DJI Matrice 210V2 (compatible with 350RTK)

---

## 🛠️ Getting Started (Software)

### 1. Prerequisites
- Familiarity with terminal and basic UNIX commands
- Recommended: Linux host (Windows/Mac supported but requires extra configuration)
- **Required Tools**
  - C++ Compiler
    - Linux: `sudo apt install build-essential`
    - Windows: Install [MinGW](https://www.mingw-w64.org/) or use WSL
    - Mac: Xcode Command Line Tools (`xcode-select --install`)
  - [VSCode](https://code.visualstudio.com/)
  - [Platform IO Extension](https://platformio.org/install/ide?install=vscode) for VSCode
  - [Optional] Install [PlatformIO Core CLI](https://docs.platformio.org/en/latest/core/quickstart.html) for terminal-only workflows
  - [Optional] Docker (https://www.docker.com/get-started/) (for running ROS & DJI SDK in containers, currently not required, for future updates)
    
### 2. Clone the Repository on the Machine Used to Deploy to ESP32
```bash
git clone https://github.com/MRU-Earth-and-Enviromental-Science/Low-Costs-Sensors-Gas.git
cd Low-Costs-Sensors-Gas
code . # Open in VSCode (or editor of your choice)
```
### 3. Upload Code to ESP32 on the Drone

- Connect your **ESP32 dev board** via USB.
- Open the Drone_System Directory on **Visual Studio Code**.
```bash
cd path-to-directory/Low-Costs-Sensors-Gas/Drone_System
``` 
- Use PlatformIO to build and upload:  
  - Click the right-facing arrow (➤) at the bottom of VSCode, or  
  - Use the command palette: `PlatformIO: Upload`  

> PlatformIO will automatically detect your environment and upload the firmware to the board.

### 4. Configure the ESP32 on Ground Station
```bash
cd "path-to-directory/Low-Costs-Sensors-Gas/Ground_Station_Reader/Ground Station"
```
- Use PlatformIO to build and upload the code to the ESP32.

### 5. Raspberry Pi Set-Up
- Running any modern Linux Distro (This was developed on Ubuntu 24.04)
- Clone this repo on the Pi
```bash
git clone https://github.com/yourusername/Low-Costs-Sensors-Gas.git
cd path-to-directory/Low-Costs-Sensors-Gas/Parser_Pi
```
- Run install_service.sh script to make it run on boot
```bash
chmod +x installService.sh
./installService.sh
```

- [Optional] Setup Docker Daemon (if wanting to use DJI OSDK)
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
### 6. Setting Up and Running the Dashboard
- Install the Dashboard from the 'Releases Section' of the GitHub repo linked below for your system.
  - For **Windows**: Download the `.exe` file.
  - For **Mac**: Download the `.dmg` file.
  - For **Linux**: Download the `.AppImage` file and make it executable:
- Alternatively, you can clone the sensor-dashboard submodule and build from source (must have npm installed):
```bash
# Clone the sensor-dashboard submodule
cd path-to-directory/Low-Costs-Sensors-Gas
git submodule update --init --recursive

cd Ground_Station_Reader/sensor-dashboard

npm install --legacy-peer-deps
npm run dev

# in a new terminal, run the following command to start the dashboard
npm run electron-dev
```
---
## 🧰 Hardware Used

- **ESP32 Dev Module (WROVER-E based)**
- **Raspberry Pi 5**
- **K30 CO₂ Sensor**
- **MQ Series Gas Sensors** (e.g., MQ-7 for CO, MQ-135 for NOₓ/CH₄)
- **Plantower PMS7003** (PM2.5 Sensor)
- **SGP30** (VOC Sensor)
- **DHT11 / DHT22** (Temperature & Humidity)
- **UART to USB Cable** (Drone to Pi connection)
- **Custom PCB** Ordered from JLC PCB (Found in Repo)

## Casing
- STEP and STL Files available
---

## 📁 Project Structure
```

/Docker/                           # Docker-based ROS environment
  ├── catkin_ws/                   # ROS workspace
  ├── .dockerignore
  ├── .env
  ├── dockerCommands.sh
  ├── Dockerfile
  ├── run.sh
  └── UserConfig.txt

/Drone_System/                     # ESP32 firmware for sensor readings
  ├── include/
  ├── lib/
  ├── src/
  ├── test/
  ├── .gitignore
  └── platformio.ini

/Ground_Station_Reader/            # Ground station receiver and dashboard
  ├── sensor-dashboard/            # Git submodule: web dashboard frontend: git@github.com:MRU-Earth-and-Enviromental-Science/sensor-dashboard.git
  └── Ground Station/
      ├── include/
      ├── lib/
      ├── src/
      ├── test/
      ├── .gitattributes
      ├── .gitignore
      └── platformio.ini

/Hardware/
  └── Gerber Files.zip/            # Gerber files for PCB

/Mechanical/
  ├── STEP-Files/                  # 3D models for casing
  └── STL-Files/                   # STEP files for custom casing

/Parser_Pi/                        # Raspberry Pi GPS parser
  ├── src/
  ├── CMakeLists.txt
  └── runOnBoot.sh
  
 .gitignore
 .gitmodules
 README.md
 LICENSE
```
---

## 📄 License
This project is licensed under the **MIT License**.

## 🛠️ Developed By

[Mount Royal University, Faculty of Science and Technology, Department of Earth and Environmental Science](https://www.mtroyal.ca/ProgramsCourses/FacultiesSchoolsCentres/ScienceTechnology/Departments/EarthEnvironmentalSciences/index.htm)

[Gwen O'Sullivan](mailto:INSERT_GWEN_EMAIL) — Vice Dean of Science and Technology @ MRU
[Shivam Walia](mailto:shivamwalia2006@gmail.com) — Mechatronics Engineering @ UWaterloo

