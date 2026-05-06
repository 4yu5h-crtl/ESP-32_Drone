/*
 * DRONE IMU TEST — Phase 2
 * ─────────────────────────────────────────────────────────
 * Reads MPU6050 and streams roll/pitch/yaw to browser
 * via WebSocket in real time.
 *
 * PINS:
 *   MPU6050 SDA → GPIO21
 *   MPU6050 SCL → GPIO22
 *   MPU6050 INT → GPIO4
 *
 * HOW TO USE:
 *   1. Fill in WiFi credentials
 *   2. Flash to ESP32
 *   3. Open browser → 192.168.29.150
 *   4. Tilt the drone and verify axes are correct:
 *      - Tilt nose DOWN  → Pitch goes POSITIVE
 *      - Tilt right side DOWN → Roll goes POSITIVE
 *      - Rotate clockwise (from above) → Yaw goes POSITIVE
 * ─────────────────────────────────────────────────────────
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ArduinoJson.h>

// ── WiFi config ───────────────────────────────────────────
const char* WIFI_SSID     = "your-wifi-ssid";       // ← change this
const char* WIFI_PASSWORD = "your-wifi-password";   // ← change this

// ── Static IP config ──────────────────────────────────────
// Change last number to any free address on your network
// e.g. if your phone shows 192.168.1.5, use 192.168.1.100
IPAddress STATIC_IP(192, 168, -, -);   // ← change to match your network
IPAddress GATEWAY  (192, 168, -, -);     // ← usually your router IP
IPAddress SUBNET   (255, 255, 255, 0);

// ── IMU pins ──────────────────────────────────────────────
#define SDA_PIN 21
#define SCL_PIN 22
#define INT_PIN  4

// ── IMU orientation correction ────────────────────────────
// Your MPU6050 is mounted upside-down on the perfboard
// These flip the axes to match real-world drone orientation
#define INVERT_X  false   // roll axis
#define INVERT_Y  false  // pitch axis
#define INVERT_Z  false  // yaw axis

// ── Complementary filter config ───────────────────────────
#define ALPHA 0.96f   // gyro weight (0.96 = smooth, reduce if drifty)

MPU6050 mpu;
WebServer server(80);
WebSocketsServer webSocket(81);

// ── State ─────────────────────────────────────────────────
float roll  = 0, pitch = 0, yaw = 0;
float gyroX_off = 0, gyroY_off = 0, gyroZ_off = 0;
float accelX_off = 0, accelY_off = 0;
unsigned long lastTime = 0;
bool calibrated = false;

// ── Calibration (runs at boot, keep drone still) ──────────
void calibrateIMU() {
  Serial.println("Calibrating IMU — keep drone STILL...");
  long gx=0, gy=0, gz=0, ax=0, ay=0;
  const int samples = 500;

  for (int i = 0; i < samples; i++) {
    int16_t ax_, ay_, az_, gx_, gy_, gz_;
    mpu.getMotion6(&ax_, &ay_, &az_, &gx_, &gy_, &gz_);
    gx += gx_; gy += gy_; gz += gz_;
    ax += ax_; ay += ay_;
    delay(2);
  }

  gyroX_off  = gx / (float)samples / 131.0f;
  gyroY_off  = gy / (float)samples / 131.0f;
  gyroZ_off  = gz / (float)samples / 131.0f;
  accelX_off = ax / (float)samples / 16384.0f;
  accelY_off = ay / (float)samples / 16384.0f;

  Serial.printf("Gyro offsets:  X=%.3f  Y=%.3f  Z=%.3f\n",
    gyroX_off, gyroY_off, gyroZ_off);
  Serial.printf("Accel offsets: X=%.3f  Y=%.3f\n",
    accelX_off, accelY_off);
  Serial.println("Calibration done.");
  calibrated = true;
}

// ── IMU update (call in loop) ─────────────────────────────
void updateIMU() {
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  lastTime = now;

  if (dt < 0.0001f || dt > 0.5f) return;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert raw to physical units
  float aX = ax / 16384.0f - accelX_off;
  float aY = ay / 16384.0f - accelY_off;
  float aZ = az / 16384.0f;
  float gX = gx / 131.0f   - gyroX_off;
  float gY = gy / 131.0f   - gyroY_off;
  float gZ = gz / 131.0f   - gyroZ_off;

  // Apply axis inversion for upside-down mount
  if (INVERT_X) { aX = -aX; gX = -gX; }
  if (INVERT_Y) { aY = -aY; gY = -gY; }
  if (INVERT_Z) { aZ = -aZ; gZ = -gZ; }

  // Accel-based angle (degrees)
  float rollAcc  = atan2(aY, aZ) * 180.0f / PI;
  float pitchAcc = atan2(-aX, sqrt(aY*aY + aZ*aZ)) * 180.0f / PI;

  // Complementary filter: blend gyro integration + accel correction
  roll  = ALPHA * (roll  + gX * dt) + (1.0f - ALPHA) * rollAcc;
  pitch = ALPHA * (pitch + gY * dt) + (1.0f - ALPHA) * pitchAcc;
  yaw  += gZ * dt;  // yaw from gyro only (no mag)

  // Keep yaw in -180 to 180
  if (yaw >  180.0f) yaw -= 360.0f;
  if (yaw < -180.0f) yaw += 360.0f;
}

// ── WebSocket event ───────────────────────────────────────
void onWebSocketEvent(uint8_t client, WStype_t type,
                      uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("Browser connected: client %d\n", client);
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("Browser disconnected: client %d\n", client);
  }
}

// ── HTML page ─────────────────────────────────────────────
const char* HTML = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Drone IMU Monitor</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: -apple-system, sans-serif;
      background: #0f0f13;
      color: #e8e6de;
      padding: 20px;
      max-width: 480px;
      margin: 0 auto;
    }
    h1 { font-size: 18px; font-weight: 500; margin-bottom: 4px; }
    .subtitle { font-size: 12px; color: #888; margin-bottom: 20px; }
    .status { font-size: 12px; margin-bottom: 20px; }
    .dot { display: inline-block; width: 8px; height: 8px;
           border-radius: 50%; margin-right: 6px; }
    .connected { background: #4ade80; }
    .disconnected { background: #f87171; }

    .angle-card {
      background: #1a1a22;
      border: 1px solid #2a2a36;
      border-radius: 10px;
      padding: 16px;
      margin-bottom: 12px;
    }
    .angle-label {
      font-size: 11px;
      color: #888;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      margin-bottom: 4px;
    }
    .angle-value {
      font-size: 42px;
      font-weight: 200;
      font-variant-numeric: tabular-nums;
      line-height: 1;
      margin-bottom: 8px;
    }
    .angle-unit { font-size: 18px; color: #888; }
    .bar-track {
      height: 6px;
      background: #2a2a36;
      border-radius: 3px;
      overflow: hidden;
    }
    .bar-fill {
      height: 100%;
      border-radius: 3px;
      transition: width 0.05s, margin-left 0.05s;
    }
    .roll-bar  { background: #f59e0b; }
    .pitch-bar { background: #4f8ef7; }
    .yaw-bar   { background: #a78bfa; }
    .hint {
      font-size: 11px;
      color: #555;
      margin-top: 6px;
    }
    .raw-section {
      background: #1a1a22;
      border: 1px solid #2a2a36;
      border-radius: 10px;
      padding: 14px;
      margin-top: 12px;
      font-family: monospace;
      font-size: 12px;
    }
    .raw-title { color: #888; margin-bottom: 8px; font-size: 11px; text-transform: uppercase; }
    .raw-row { display: flex; justify-content: space-between; margin-bottom: 4px; }
    .raw-key { color: #666; }
    .raw-val { color: #e8e6de; }
    .calibration-box {
      background: #1a2a1a;
      border-left: 3px solid #4ade80;
      padding: 10px 14px;
      border-radius: 0 6px 6px 0;
      font-size: 12px;
      color: #86efac;
      margin-bottom: 16px;
    }
    .verify-box {
      background: #1a1a2a;
      border: 1px solid #3a3a5a;
      border-radius: 10px;
      padding: 14px;
      margin-top: 12px;
      font-size: 12px;
      color: #a5b4fc;
    }
    .verify-box h3 { font-size: 13px; margin-bottom: 8px; color: #c7d2fe; }
    .verify-row { margin-bottom: 6px; color: #818cf8; }
    .btn {
      width: 100%;
      padding: 12px;
      background: #1e3a5f;
      border: 1px solid #2d5a8e;
      color: #60a5fa;
      border-radius: 8px;
      font-size: 14px;
      cursor: pointer;
      margin-top: 12px;
    }
    .btn:active { background: #2d5a8e; }
  </style>
</head>
<body>
  <h1>📡 IMU Monitor</h1>
  <p class="subtitle">Phase 2 — Live roll / pitch / yaw</p>

  <div class="status">
    <span class="dot disconnected" id="dot"></span>
    <span id="statusText">Connecting...</span>
  </div>

  <div class="calibration-box" id="calBox">
    ⏳ Waiting for calibration to complete...
  </div>

  <!-- Roll -->
  <div class="angle-card">
    <div class="angle-label">Roll (X axis)</div>
    <div class="angle-value" id="roll">0.0 <span class="angle-unit">°</span></div>
    <div class="bar-track">
      <div class="bar-fill roll-bar" id="rollBar" style="width:50%;margin-left:0%"></div>
    </div>
    <div class="hint">Tilt right side down → positive</div>
  </div>

  <!-- Pitch -->
  <div class="angle-card">
    <div class="angle-label">Pitch (Y axis)</div>
    <div class="angle-value" id="pitch">0.0 <span class="angle-unit">°</span></div>
    <div class="bar-track">
      <div class="bar-fill pitch-bar" id="pitchBar" style="width:50%;margin-left:0%"></div>
    </div>
    <div class="hint">Tilt nose down → positive</div>
  </div>

  <!-- Yaw -->
  <div class="angle-card">
    <div class="angle-label">Yaw (Z axis)</div>
    <div class="angle-value" id="yaw">0.0 <span class="angle-unit">°</span></div>
    <div class="bar-track">
      <div class="bar-fill yaw-bar" id="yawBar" style="width:50%;margin-left:0%"></div>
    </div>
    <div class="hint">Rotate clockwise from above → positive</div>
  </div>

  <!-- Raw values -->
  <div class="raw-section">
    <div class="raw-title">Raw sensor data</div>
    <div class="raw-row"><span class="raw-key">Accel X</span><span class="raw-val" id="rax">-</span></div>
    <div class="raw-row"><span class="raw-key">Accel Y</span><span class="raw-val" id="ray">-</span></div>
    <div class="raw-row"><span class="raw-key">Accel Z</span><span class="raw-val" id="raz">-</span></div>
    <div class="raw-row"><span class="raw-key">Gyro X</span><span class="raw-val" id="rgx">-</span></div>
    <div class="raw-row"><span class="raw-key">Gyro Y</span><span class="raw-val" id="rgy">-</span></div>
    <div class="raw-row"><span class="raw-key">Gyro Z</span><span class="raw-val" id="rgz">-</span></div>
    <div class="raw-row"><span class="raw-key">Loop dt</span><span class="raw-val" id="rdt">-</span></div>
  </div>

  <!-- Axis verification guide -->
  <div class="verify-box">
    <h3>✅ Axis verification checklist</h3>
    <div class="verify-row">1. Lay flat → Roll≈0, Pitch≈0</div>
    <div class="verify-row">2. Tilt nose DOWN → Pitch goes positive ↑</div>
    <div class="verify-row">3. Tilt RIGHT side down → Roll goes positive ↑</div>
    <div class="verify-row">4. Rotate CW from above → Yaw goes positive ↑</div>
    <div class="verify-row">If any axis is inverted → tell me which one</div>
  </div>

  <button class="btn" onclick="resetYaw()">Reset Yaw to 0</button>

  <script>
    const ws = new WebSocket('ws://' + location.hostname + ':81');

    ws.onopen = () => {
      document.getElementById('dot').className = 'dot connected';
      document.getElementById('statusText').textContent = 'Connected — receiving IMU data';
    };
    ws.onclose = () => {
      document.getElementById('dot').className = 'dot disconnected';
      document.getElementById('statusText').textContent = 'Disconnected';
    };

    ws.onmessage = (e) => {
      const d = JSON.parse(e.data);

      if (d.calibrated) {
        document.getElementById('calBox').textContent = '✅ Calibration complete — drone was kept still';
        document.getElementById('calBox').style.background = '#1a2a1a';
      }

      // Update angle displays
      setAngle('roll',  d.roll);
      setAngle('pitch', d.pitch);
      setAngle('yaw',   d.yaw);

      // Raw values
      document.getElementById('rax').textContent = d.ax?.toFixed(3);
      document.getElementById('ray').textContent = d.ay?.toFixed(3);
      document.getElementById('raz').textContent = d.az?.toFixed(3);
      document.getElementById('rgx').textContent = d.gx?.toFixed(2) + ' °/s';
      document.getElementById('rgy').textContent = d.gy?.toFixed(2) + ' °/s';
      document.getElementById('rgz').textContent = d.gz?.toFixed(2) + ' °/s';
      document.getElementById('rdt').textContent = d.dt?.toFixed(1) + ' ms';
    };

    function setAngle(id, val) {
      document.getElementById(id).innerHTML =
        val.toFixed(1) + ' <span class="angle-unit">°</span>';
      // Bar: map -90..90 to 0..100%
      const pct = Math.min(100, Math.max(0, (val + 90) / 180 * 100));
      document.getElementById(id+'Bar').style.width = '4px';
      document.getElementById(id+'Bar').style.marginLeft = pct + '%';
    }

    function resetYaw() {
      fetch('/resetyaw');
    }
  </script>
</body>
</html>
)rawhtml";

// ── HTTP handlers ─────────────────────────────────────────
void handleRoot() {
  server.send(200, "text/html", HTML);
}

void handleResetYaw() {
  yaw = 0;
  server.send(200, "text/plain", "OK");
}

// ── Setup ─────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\nDrone IMU Test — Phase 2");

  // Init I2C + MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 NOT detected — check wiring!");
    while (1) delay(500);
  }
  Serial.println("MPU6050 detected OK");

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);  // low pass filter

  // Connect WiFi
  WiFi.config(STATIC_IP, GATEWAY, SUBNET);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setHostname("esp32-drone");

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());

  // Start servers
  server.on("/",         handleRoot);
  server.on("/resetyaw", handleResetYaw);
  server.begin();

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  Serial.println("Open browser at: http://192.168.29.150");

  // Calibrate IMU (keep drone still for ~1 second)
  delay(500);
  calibrateIMU();
  lastTime = micros();
}

// ── Loop ──────────────────────────────────────────────────
unsigned long lastSend = 0;

void loop() {
  server.handleClient();
  webSocket.loop();

  updateIMU();

  // Send data to browser at 20Hz
  if (millis() - lastSend > 50) {
    lastSend = millis();

    // Get fresh raw values for display
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float aXd = (INVERT_X ? -1 : 1) * (ax / 16384.0f - accelX_off);
    float aYd = (INVERT_Y ? -1 : 1) * (ay / 16384.0f - accelY_off);
    float aZd = az / 16384.0f;
    float gXd = (INVERT_X ? -1 : 1) * (gx / 131.0f - gyroX_off);
    float gYd = (INVERT_Y ? -1 : 1) * (gy / 131.0f - gyroY_off);
    float gZd = gz / 131.0f - gyroZ_off;

    StaticJsonDocument<256> doc;
    doc["roll"]      = round(roll  * 10) / 10.0;
    doc["pitch"]     = round(pitch * 10) / 10.0;
    doc["yaw"]       = round(yaw   * 10) / 10.0;
    doc["ax"]        = aXd;
    doc["ay"]        = aYd;
    doc["az"]        = aZd;
    doc["gx"]        = gXd;
    doc["gy"]        = gYd;
    doc["gz"]        = gZd;
    doc["dt"]        = (micros() - lastTime) / 1000.0f;
    doc["calibrated"] = calibrated;

    String json;
    serializeJson(doc, json);
    webSocket.broadcastTXT(json);
  }
}
