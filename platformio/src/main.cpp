#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <Wire.h>

// initialization
// temp
#define DHTPIN 4
#define DHTTYPE DHT11
// ch4
#define MQ4_PIN 34
#define RL_VALUE 10.0
#define RO_CLEAN_AIR_FACTOR 4.4
// SDA and SCL
#define SDA_pin 27
#define SCL_pin 26
#define K30_ADDRESS 0x68
// wifi
const char ssid[] = "gasSensor";
const char password[] = "Neon2017";

// global
WebServer server(80);
DHT dht(DHTPIN, DHTTYPE);
float Ro = 10.0;

// K30 CO₂
class K30_I2C
{
public:
  K30_I2C(int i2c_address) { _i2c_address = i2c_address; }
  int readCO2(int &CO2level)
  {
    byte recValue[4] = {0};
    Wire.beginTransmission(_i2c_address);
    Wire.write(0x22);
    Wire.write(0x00);
    Wire.write(0x08);
    Wire.write(0x2A);
    Wire.endTransmission();
    delay(30);
    Wire.requestFrom(_i2c_address, 4);
    delay(20);
    byte i = 0;
    while (Wire.available() && i < 4)
      recValue[i++] = Wire.read();
    CO2level = (recValue[1] << 8) + recValue[2];
    byte checkSum = recValue[0] + recValue[1] + recValue[2];
    if (i == 0)
      return 2;
    else if (checkSum == recValue[3])
      return 0;
    else
      return 1;
  }

private:
  int _i2c_address;
};

K30_I2C k30(K30_ADDRESS);

// functions
void initWifi();
void sendData();
float readMQ4();
float calculateResistance(int adcValue);
float calibrateSensor();

// setup
void setup()
{
  Serial.begin(9600);
  Wire.begin(SDA_pin, SCL_pin);
  dht.begin();
  Ro = calibrateSensor();
  Serial.print("Calibrated Ro = ");
  Serial.println(Ro);
  initWifi();
}

// main loop
void loop()
{
  server.handleClient();
}

const char *htmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">

<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Sensor Dashboard</title>
  <style>
    :root {
      --bg-primary: #000000;
      --bg-secondary: #000000;
      --bg-tertiary: #000000;
      --text-primary: #ffffff;
      --text-secondary: #ffffff;
      --text-muted: #888888;
      --accent: #ffffff;
      --accent-hover: #e0e0e0;
      --shadow-sm: 0 4px 6px rgba(0, 0, 0, 0.1);
      --shadow-md: 0 8px 24px rgba(0, 0, 0, 0.2);
      --shadow-lg: 0 12px 32px rgba(0, 0, 0, 0.3);
      --border-radius: 8px;
    }

    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
    }

    body {
      font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
      background: var(--bg-primary);
      color: var(--text-primary);
      margin: 0;
      padding: 2rem;
      display: flex;
      flex-direction: column;
      align-items: center;
      min-height: 100vh;
    }

    h1 {
      font-size: 2.5rem;
      font-weight: 700;
      margin-bottom: 2rem;
      color: var(--text-primary);
      text-align: center;
      letter-spacing: -0.5px;
    }

    .dashboard-container {
      width: 100%;
      max-width: 850px;
      display: flex;
      flex-direction: column;
      gap: 1.5rem;
    }

    .controls {
      display: flex;
      flex-wrap: wrap;
      gap: 0.75rem;
      margin-bottom: 0.5rem;
      justify-content: center;
    }

    button {
      background-color: var(--bg-tertiary);
      border: 1px solid #333;
      color: var(--text-primary);
      padding: 0.75rem 1.5rem;
      font-size: 1rem;
      font-weight: 500;
      border-radius: var(--border-radius);
      cursor: pointer;
      box-shadow: var(--shadow-sm);
      transition: all 0.2s ease;
      min-width: 120px;
    }

    button:hover {
      background-color: var(--accent);
      color: var(--bg-primary);
      box-shadow: var(--shadow-md);
      transform: translateY(-2px);
    }

    button:active {
      transform: translateY(0);
      box-shadow: var(--shadow-sm);
    }

    .log {
      width: 100%;
      background: var(--bg-secondary);
      border-radius: var(--border-radius);
      padding: 1.25rem;
      box-shadow: var(--shadow-md);
      height: 350px;
      overflow-y: auto;
      white-space: pre-line;
      font-family: 'Courier New', monospace;
      font-size: 0.95rem;
      color: var(--text-secondary);
      border: 1px solid #333;
      transition: box-shadow 0.3s ease;
    }

    .log:hover {
      box-shadow: var(--shadow-lg);
    }

    .footer {
      margin-top: 2.5rem;
      font-size: 0.9rem;
      color: var(--text-muted);
      text-align: center;
      width: 100%;
      max-width: 850px;
      padding: 1rem;
      border-top: 1px solid #333;
    }

    .footer strong {
      color: var(--text-primary);
      font-weight: 600;
    }

    /* Custom scrollbar */
    .log::-webkit-scrollbar {
      width: 8px;
    }

    .log::-webkit-scrollbar-track {
      background: var(--bg-secondary);
      border-radius: var(--border-radius);
    }

    .log::-webkit-scrollbar-thumb {
      background: #444;
      border-radius: var(--border-radius);
    }

    .log::-webkit-scrollbar-thumb:hover {
      background: #555;
    }

    @media (max-width: 600px) {
      body {
        padding: 1rem;
      }

      h1 {
        font-size: 1.75rem;
        margin-bottom: 1.5rem;
      }

      .controls {
        flex-direction: column;
        width: 100%;
      }

      button {
        width: 100%;
      }
    }
  </style>
</head>

<body>
  <h1>📡 Sensor Dashboard</h1>

  <div class="dashboard-container">
    <div class="controls">
      <button onclick="startLogging()">Start</button>
      <button onclick="stopLogging()">Stop</button>
      <button onclick="downloadCSV()">Download CSV</button>
    </div>

    <div class="log" id="logDisplay">Waiting for data...</div>
  </div>

  <div class="footer" style="padding:20px;">
    <p><strong><a
          href="https://www.mtroyal.ca/ProgramsCourses/FacultiesSchoolsCentres/ScienceTechnology/Departments/EarthEnvironmentalSciences/index.htm"
          target="_blank">Mount Royal University © 2025</a></strong></p>
    <div style="height: 5px;"></div>
    <p>🛠 developed by shivam walia, mechatronics @uwaterloo '29 <a
        href="https://www.linkedin.com/in/shivam-walia-395877251/" target="_blank"
        style="color: #1e90ff; text-decoration: none;">
        [linkedIn]<a href="https://github.com/shivam-2507" target="_blank"
          style="color: #1e90ff; text-decoration: none;">
          [github]</p>
  </div>

  <script>
    let logging = false;
    let dataLog = [["Timestamp", "Temperature", "Humidity", "CH4", "CO2"]];
    const display = document.getElementById("logDisplay");

    function startLogging() {
      if (!logging) {
        logging = true;
        display.textContent = "Starting data collection...\n";
        poll();
      }
    }

    function stopLogging() {
      if (logging) {
        logging = false;
        display.textContent += "\nData collection stopped.\n";
      }
    }

    function downloadCSV() {
      const csv = dataLog.map(row => row.join(",")).join("\n");
      const blob = new Blob([csv], { type: "text/csv" });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = "esp32_log_" + new Date().toISOString().slice(0, 10) + ".csv";
      a.click();
    }

    function poll() {
      if (!logging) return;
      fetch("/data")
        .then(res => res.json())
        .then(data => {
          const now = new Date().toLocaleTimeString();
          const row = [
            now,
            data.temperature,
            data.humidity,
            data.ch4_ppm,
            data.co2_ppm
          ];
          dataLog.push(row);
          display.textContent += `\n${now} | T=${data.temperature}°C | H=${data.humidity}% | CH₄=${data.ch4_ppm}ppm | CO₂=${data.co2_ppm}ppm`;
          display.scrollTop = display.scrollHeight;
        })
        .catch(err => {
          console.error("Fetch error:", err);
          display.textContent += "\nError fetching data. Retrying...";
        });
      setTimeout(poll, 1000);
    }
  </script>
</body>

</html>
)rawliteral";

// wifi initialization
void initWifi()
{
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point started. IP: ");
  Serial.println(IP);

  server.on("/", HTTP_GET, []()
            { server.send(200, "text/html", htmlPage); });

  server.on("/data", HTTP_GET, sendData);

  server.begin();
  Serial.println("Web server started");
}

// data export
void sendData()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float ch4 = readMQ4();
  int co2ppm = 0;
  int co2status = k30.readCO2(co2ppm);

  if (isnan(h) || isnan(t))
  {
    server.send(500, "application/json", "{\"error\": \"DHT read failed\"}");
    return;
  }
  if (co2status == 1)
  {
    server.send(500, "application/json", "{\"error\": \"CO2 read failed\"}");
    return;
  }

  String json = "{";
  json += "\"temperature\":" + String(t, 2) + ",";
  json += "\"humidity\":" + String(h, 2) + ",";
  json += "\"ch4_ppm\":" + String(ch4, 2) + ",";
  json += "\"co2_ppm\":" + String(co2ppm);
  json += "}";

  server.send(200, "application/json", json);
}

// read CH4
float readMQ4()
{
  int adcValue = analogRead(MQ4_PIN);
  float Rs = calculateResistance(adcValue);
  float ratio = Rs / Ro;
  float m = -0.318;
  float b = 1.133;
  float ppm_log = (log10(ratio) - b) / m;
  return pow(10, ppm_log);
}

float calculateResistance(int adcValue)
{
  float voltage = adcValue * (3.3 / 4095.0);
  return RL_VALUE * (3.3 - voltage) / voltage;
}

float calibrateSensor()
{
  float val = 0.0;
  for (int i = 0; i < 50; i++)
  {
    val += calculateResistance(analogRead(MQ4_PIN));
    delay(100);
  }
  val /= 50.0;
  return val / RO_CLEAN_AIR_FACTOR;
}