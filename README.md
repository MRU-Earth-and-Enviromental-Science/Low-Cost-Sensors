# 📡 Serial Data Protocol
The system sends processed sensor data over serial using a flag-based protocol for easy identification and parsing:

#### **Data Format Structure:**
```
[FLAG_BYTE][DATA_PAYLOAD]
```

#### **Flag Types:**
- **`0x10`** → Binary Data Format (Structured Packet)
- **`0x20`** → String Data Format (JSON)

#### **Binary Format (`0x10`):**
When the flag is `0x10`, the following binary packet structure is transmitted:
```c
struct SensorPacket {
    uint8_t header[4];        // "SENS" 
    uint32_t timestamp;       // Unix timestamp
    float temperature;        // °C
    float humidity;          // %
    float ch4;               // ppm
    float co2;               // ppm  
    float tvoc;              // ppb
    float co;                // ppm
    float nox;               // ppm
    float pm1;               // µg/m³
    float pm25;              // µg/m³
    float pm10;              // µg/m³
    uint16_t checksum;       // Data integrity check
    uint8_t footer[2];       // "\r\n"
};
```

#### **String Format (`0x20`):**
When the flag is `0x20`, a JSON string follows:
```json
{
  "timestamp": 1691234567,
  "temperature": 25.50,
  "humidity": 60.20,
  "ch4": 1.05,
  "co2": 450.30,
  "tvoc": 125.80,
  "co": 2.10,
  "nox": 0.85,
  "pm1": 12.40,
  "pm25": 15.60,
  "pm10": 18.90
}
```

#### **How to Decode:**
1. **Read the first byte** to identify the format
2. **If `0x10`:** Read 58 bytes for the complete binary packet, validate header/footer/checksum
3. **If `0x20`:** Read characters until newline (`\n`) for the JSON string
4. **Parse accordingly** based on the format detected

#### **Configuration for RPi:**
- **Port:** `/dev/ttyUSB1` (configurable via ROS parameter `serial_port`)
- **Baud Rate:** `115200` (configurable via ROS parameter `baud_rate`)  
- **Format:** Set via ROS parameter `binary_format` (true=binary, false=string)



# Ignore below this
An air quality monitoring system designed for DJI Matrice drones, capable of measuring and logging key environmental parameters: CO<sub>2</sub>, CO, CH<sub>4</sub>, NO<sub>x</sub>, PM<sub>2.5</sub>, VOCs, temperature, and humidity. 

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
  - CH<sub>4</sub> in ppm
  - NO<sub>x</sub> in ppm
  - PM2.5 particulate matter in μg/m<sup>3</sup>
  - VOCs (Volatile Organic Compounds)
  - Temperature (°C) and Humidity (%)
- PlatformIO-based development environment.
- ROS Noetic Pi-ESP communication (via ROS Serial)

## 👨‍💻 Software Overview
- ESP32 gathers sensor data
- Raspberry Pi receives sensor data from the ESP32 over ROSSerial
- Raspberry Pi is able to send data to an open serial port (/dev/ttyUSB1) to export data elsewhere

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



## 📄 License
This project is licensed under the **MIT License**.

## 🛠️ Developed By

[Mount Royal University, Faculty of Science and Technology, Department of Earth and Environmental Science](https://www.mtroyal.ca/ProgramsCourses/FacultiesSchoolsCentres/ScienceTechnology/Departments/EarthEnvironmentalSciences/index.htm)

[Gwen O'Sullivan](mailto:INSERT_GWEN_EMAIL) — Vice Dean of Science and Technology @ MRU
[Shivam Walia](mailto:shivamwalia2006@gmail.com) — Mechatronics Engineering @ UWaterloo

