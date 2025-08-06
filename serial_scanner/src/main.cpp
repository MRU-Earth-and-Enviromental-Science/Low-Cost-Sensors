#include <Arduino.h>

// Sensor data structure to match the sender
struct __attribute__((packed)) SensorPacket {
    uint8_t header[4];        // Packet header: "SENS"
    uint32_t timestamp;       // 4 bytes
    float temperature;        // 4 bytes
    float humidity;          // 4 bytes
    float ch4;               // 4 bytes
    float co2;               // 4 bytes
    float tvoc;              // 4 bytes
    float co;                // 4 bytes
    float nox;               // 4 bytes
    float pm1;               // 4 bytes
    float pm25;              // 4 bytes
    float pm10;              // 4 bytes
    uint16_t checksum;       // 2 bytes CRC
    uint8_t footer[2];       // Packet footer: "\r\n"
};

// Function declarations
bool processBinaryData();
bool processStringData();
bool validateBinaryPacket(const SensorPacket& packet);
uint16_t calculateChecksum(const SensorPacket& packet);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  Serial.println("=== Sensor Data Receiver Started ===");
  Serial.println("Waiting for data...");
}

void loop() {
  if (Serial.available() > 0) {
    uint8_t flag = Serial.read();
    
    if (flag == 0x10) {
      // Binary data format
      Serial.println("\n--- Binary Data Received (0x10) ---");
      Serial.print("Time: "); Serial.println(millis());
      bool success = processBinaryData();
      
      // Send acknowledgment
      if (success) {
        Serial.println("✓ BINARY DATA SUCCESSFULLY RECEIVED AND VALIDATED");
        // Send ACK back to sender (optional)
        Serial.write(0xAA); // ACK byte for binary
      } else {
        Serial.println("✗ BINARY DATA RECEPTION FAILED");
        // Send NACK back to sender (optional)
        Serial.write(0xBB); // NACK byte for binary
      }
    } 
    else if (flag == 0x20) {
      // String data format
      Serial.println("\n--- String Data Received (0x20) ---");
      Serial.print("Time: "); Serial.println(millis());
      bool success = processStringData();
      
      // Send acknowledgment
      if (success) {
        Serial.println("✓ STRING DATA SUCCESSFULLY RECEIVED AND PARSED");
        // Send ACK back to sender (optional)
        Serial.write(0xCC); // ACK byte for string
      } else {
        Serial.println("✗ STRING DATA RECEPTION FAILED");
        // Send NACK back to sender (optional)
        Serial.write(0xDD); // NACK byte for string
      }
    } 
    else {
      // Unknown flag - might be part of string data or noise
      Serial.print("Unknown flag: 0x");
      Serial.print(flag, HEX);
      Serial.print(" at time: ");
      Serial.println(millis());
    }
  }
}

bool processBinaryData() {
  // Wait for enough bytes for a complete packet
  const size_t packetSize = sizeof(SensorPacket);
  
  Serial.print("Waiting for "); Serial.print(packetSize); Serial.println(" bytes...");
  
  // Wait up to 1 second for complete packet
  unsigned long timeout = millis() + 1000;
  while (Serial.available() < packetSize && millis() < timeout) {
    delay(1);
  }
  
  if (Serial.available() < packetSize) {
    Serial.print("Timeout! Only received "); 
    Serial.print(Serial.available()); 
    Serial.print(" of "); 
    Serial.print(packetSize); 
    Serial.println(" bytes");
    return false;
  }
  
  Serial.print("Received "); Serial.print(packetSize); Serial.println(" bytes");
  
  // Read the binary packet
  SensorPacket packet;
  size_t bytesRead = Serial.readBytes((uint8_t*)&packet, packetSize);
  
  Serial.print("Actually read "); Serial.print(bytesRead); Serial.println(" bytes");
  
  // Validate packet
  if (!validateBinaryPacket(packet)) {
    Serial.println("Validation failed!");
    return false;
  }
  
  Serial.println("Validation passed!");
  
  // Print the data
  Serial.println("Binary Sensor Data:");
  Serial.print("  Header: ");
  for (int i = 0; i < 4; i++) {
    Serial.print((char)packet.header[i]);
  }
  Serial.println();
  
  Serial.print("  Timestamp: "); Serial.println(packet.timestamp);
  Serial.print("  Temperature: "); Serial.print(packet.temperature); Serial.println(" °C");
  Serial.print("  Humidity: "); Serial.print(packet.humidity); Serial.println(" %");
  Serial.print("  CH4: "); Serial.print(packet.ch4); Serial.println(" ppm");
  Serial.print("  CO2: "); Serial.print(packet.co2); Serial.println(" ppm");
  Serial.print("  TVOC: "); Serial.print(packet.tvoc); Serial.println(" ppb");
  Serial.print("  CO: "); Serial.print(packet.co); Serial.println(" ppm");
  Serial.print("  NOx: "); Serial.print(packet.nox); Serial.println(" ppm");
  Serial.print("  PM1.0: "); Serial.print(packet.pm1); Serial.println(" µg/m³");
  Serial.print("  PM2.5: "); Serial.print(packet.pm25); Serial.println(" µg/m³");
  Serial.print("  PM10.0: "); Serial.print(packet.pm10); Serial.println(" µg/m³");
  Serial.print("  Checksum: 0x"); Serial.println(packet.checksum, HEX);
  
  return true;
}

bool processStringData() {
  // Read string data until newline or timeout
  String jsonData = "";
  unsigned long timeout = millis() + 1000;
  int bytesReceived = 0;
  
  Serial.println("Reading string data...");
  
  while (millis() < timeout) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      bytesReceived++;
      if (c == '\n') {
        Serial.print("String complete! Received "); 
        Serial.print(bytesReceived); 
        Serial.println(" bytes");
        break; // End of JSON string
      }
      jsonData += c;
    }
  }
  
  if (jsonData.length() == 0) {
    Serial.println("No string data received (timeout or empty)");
    return false;
  }
  
  Serial.print("Total string length: "); Serial.println(jsonData.length());
  
  // Print the JSON data
  Serial.println("String Sensor Data (JSON):");
  Serial.println(jsonData);
  
  // Simple parsing for display (basic JSON parsing)
  if (jsonData.indexOf("timestamp") != -1) {
    Serial.println("Parsed Values:");
    
    // Extract values (simple string parsing)
    int idx;
    
    // Timestamp
    idx = jsonData.indexOf("\"timestamp\":");
    if (idx != -1) {
      int start = jsonData.indexOf(':', idx) + 1;
      int end = jsonData.indexOf(',', start);
      if (end == -1) end = jsonData.indexOf('}', start);
      Serial.print("  Timestamp: ");
      Serial.println(jsonData.substring(start, end));
    }
    
    // Temperature
    idx = jsonData.indexOf("\"temperature\":");
    if (idx != -1) {
      int start = jsonData.indexOf(':', idx) + 1;
      int end = jsonData.indexOf(',', start);
      Serial.print("  Temperature: ");
      Serial.print(jsonData.substring(start, end));
      Serial.println(" °C");
    }
    
    // Humidity
    idx = jsonData.indexOf("\"humidity\":");
    if (idx != -1) {
      int start = jsonData.indexOf(':', idx) + 1;
      int end = jsonData.indexOf(',', start);
      Serial.print("  Humidity: ");
      Serial.print(jsonData.substring(start, end));
      Serial.println(" %");
    }
    
    // Add more parsing as needed...
    Serial.println("  [Additional sensors parsed similarly...]");
    return true;
  } else {
    Serial.println("JSON parsing failed - no timestamp found");
    return false;
  }
}

bool validateBinaryPacket(const SensorPacket& packet) {
  Serial.println("Validating packet...");
  
  // Check header
  if (packet.header[0] != 'S' || packet.header[1] != 'E' || 
      packet.header[2] != 'N' || packet.header[3] != 'S') {
    Serial.print("Invalid header: ");
    for (int i = 0; i < 4; i++) {
      Serial.print((char)packet.header[i]);
    }
    Serial.println();
    return false;
  }
  Serial.println("Header OK");
  
  // Check footer
  if (packet.footer[0] != '\r' || packet.footer[1] != '\n') {
    Serial.print("Invalid footer: 0x");
    Serial.print(packet.footer[0], HEX);
    Serial.print(" 0x");
    Serial.println(packet.footer[1], HEX);
    return false;
  }
  Serial.println("Footer OK");
  
  // Verify checksum
  uint16_t calculatedChecksum = calculateChecksum(packet);
  if (calculatedChecksum != packet.checksum) {
    Serial.print("Checksum mismatch. Calculated: 0x");
    Serial.print(calculatedChecksum, HEX);
    Serial.print(", Received: 0x");
    Serial.println(packet.checksum, HEX);
    return false;
  }
  Serial.println("Checksum OK");
  
  return true;
}

uint16_t calculateChecksum(const SensorPacket& packet) {
  uint16_t checksum = 0;
  const uint8_t* data_ptr = reinterpret_cast<const uint8_t*>(&packet.timestamp);
  
  // Calculate checksum for data portion (excluding header, checksum, and footer)
  size_t dataSize = sizeof(SensorPacket) - sizeof(packet.header) - sizeof(packet.checksum) - sizeof(packet.footer);
  
  for (size_t i = 0; i < dataSize; i++) {
    checksum += data_ptr[i];
  }
  
  return checksum;
}