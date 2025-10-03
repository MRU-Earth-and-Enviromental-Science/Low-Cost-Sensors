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

