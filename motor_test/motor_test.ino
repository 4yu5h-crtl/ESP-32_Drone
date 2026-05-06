/*
 * DRONE MOTOR TEST — Phase 1
 * ─────────────────────────────────────────────────────────
 * Connects to home WiFi, gets static IP, serves a webpage
 * with 4 motor sliders for individual motor testing.
 *
 * PINS:
 *   M1 (Front-right  CW)  → GPIO25
 *   M2 (Rear-left    CW)  → GPIO26
 *   M3 (Rear-right   CCW) → GPIO27
 *   M4 (Front-left   CCW) → GPIO14
 *
 * HOW TO USE:
 *   1. Fill in your WiFi credentials below
 *   2. Set STATIC_IP to a free IP on your network
 *   3. Flash to ESP32
 *   4. Open browser on phone/PC → go to the static IP
 *   5. Use sliders to test each motor individually
 *      NO PROPELLERS during this test!
 * ─────────────────────────────────────────────────────────
 */

#include <WiFi.h>
#include <WebServer.h>
#include "driver/ledc.h"

// ── WiFi credentials ──────────────────────────────────────
const char* WIFI_SSID     = "your-wifi-ssid";       // ← change this
const char* WIFI_PASSWORD = "your-wifi-password";   // ← change this

// ── Static IP config ──────────────────────────────────────
// Change last number to any free address on your network
// e.g. if your phone shows 192.168.1.5, use 192.168.1.100
IPAddress STATIC_IP(192, 168, -, -);   // ← change to match your network
IPAddress GATEWAY  (192, 168, -, -);     // ← usually your router IP
IPAddress SUBNET   (255, 255, 255, 0);

// ── Motor pins ────────────────────────────────────────────
#define M1_PIN  25    // Front-right  CW
#define M2_PIN  26    // Rear-left    CW
#define M3_PIN  27    // Rear-right   CCW
#define M4_PIN  14    // Front-left   CCW

// ── PWM config ────────────────────────────────────────────
#define PWM_FREQ    25000   // 25kHz for brushed motors
#define PWM_RES     8       // 8-bit: 0–255
#define MAX_SAFE    180     // Safety cap during testing (70%)

WebServer server(80);

int MOTOR_PINS[4] = {M1_PIN, M2_PIN, M3_PIN, M4_PIN};

void setMotor(int motorIndex, int value) {
  value = constrain(value, 0, MAX_SAFE);
  ledcWrite(MOTOR_PINS[motorIndex], value);
}

void stopAllMotors() {
  for (int i = 0; i < 4; i++) ledcWrite(MOTOR_PINS[i], 0);
}

// ── Web page HTML ─────────────────────────────────────────
const char* HTML = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Drone Motor Test</title>
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
    h1 { font-size: 18px; font-weight: 500; margin-bottom: 4px; color: #fff; }
    .subtitle { font-size: 12px; color: #888; margin-bottom: 24px; }
    .warning {
      background: #2a1a00;
      border-left: 3px solid #f59e0b;
      padding: 10px 14px;
      border-radius: 0 6px 6px 0;
      font-size: 12px;
      color: #fcd34d;
      margin-bottom: 24px;
    }
    .motor-card {
      background: #1a1a22;
      border: 1px solid #2a2a36;
      border-radius: 10px;
      padding: 16px;
      margin-bottom: 14px;
    }
    .motor-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 12px;
    }
    .motor-name { font-size: 14px; font-weight: 500; }
    .motor-badge {
      font-size: 11px;
      padding: 2px 8px;
      border-radius: 4px;
      font-weight: 500;
    }
    .cw  { background: #1a3a2a; color: #4ade80; }
    .ccw { background: #1a2a3a; color: #60a5fa; }
    .motor-value {
      font-size: 28px;
      font-weight: 300;
      color: #fff;
      margin-bottom: 10px;
      font-variant-numeric: tabular-nums;
    }
    .motor-pct { font-size: 14px; color: #888; }
    input[type=range] {
      width: 100%;
      height: 6px;
      border-radius: 3px;
      outline: none;
      -webkit-appearance: none;
      background: #2a2a36;
      margin-bottom: 10px;
    }
    input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 22px;
      height: 22px;
      border-radius: 50%;
      background: #4f8ef7;
      cursor: pointer;
    }
    .stop-btn {
      width: 100%;
      padding: 8px;
      background: #2a1a1a;
      border: 1px solid #4a2020;
      color: #f87171;
      border-radius: 6px;
      font-size: 12px;
      cursor: pointer;
    }
    .stop-btn:active { background: #4a2020; }
    .stop-all {
      width: 100%;
      padding: 14px;
      background: #7f1d1d;
      border: none;
      color: #fff;
      border-radius: 10px;
      font-size: 16px;
      font-weight: 500;
      cursor: pointer;
      margin-top: 8px;
    }
    .stop-all:active { background: #991b1b; }
    .status {
      text-align: center;
      font-size: 12px;
      color: #4ade80;
      margin-top: 16px;
      height: 20px;
    }
    .layout {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      background: #1a1a22;
      border: 1px solid #2a2a36;
      border-radius: 10px;
      padding: 14px;
      margin-bottom: 20px;
      font-size: 11px;
      color: #888;
      text-align: center;
    }
    .layout-cell { padding: 6px; border-radius: 6px; background: #22222e; }
    .layout-cell span { display: block; font-size: 13px; font-weight: 500; color: #e8e6de; margin-top: 2px; }
  </style>
</head>
<body>
  <h1>🚁 Motor Test</h1>
  <p class="subtitle">Phase 1 — Individual motor verification</p>

  <div class="warning">⚠ Remove all propellers before testing</div>

  <div class="layout">
    <div class="layout-cell">M4 Front-left<span>CCW 🔵</span></div>
    <div class="layout-cell">M1 Front-right<span>CW 🟢</span></div>
    <div class="layout-cell">M2 Rear-left<span>CW 🟢</span></div>
    <div class="layout-cell">M3 Rear-right<span>CCW 🔵</span></div>
  </div>

  <div class="motor-card">
    <div class="motor-header">
      <span class="motor-name">M1 — Front-right</span>
      <span class="motor-badge cw">CW · GPIO25</span>
    </div>
    <div class="motor-value" id="v1">0 <span class="motor-pct">/ 255</span></div>
    <input type="range" min="0" max="180" value="0"
      oninput="send(1,this.value)" onchange="send(1,this.value)" id="s1">
    <button class="stop-btn" onclick="stopMotor(1)">Stop M1</button>
  </div>

  <div class="motor-card">
    <div class="motor-header">
      <span class="motor-name">M2 — Rear-left</span>
      <span class="motor-badge cw">CW · GPIO26</span>
    </div>
    <div class="motor-value" id="v2">0 <span class="motor-pct">/ 255</span></div>
    <input type="range" min="0" max="180" value="0"
      oninput="send(2,this.value)" onchange="send(2,this.value)" id="s2">
    <button class="stop-btn" onclick="stopMotor(2)">Stop M2</button>
  </div>

  <div class="motor-card">
    <div class="motor-header">
      <span class="motor-name">M3 — Rear-right</span>
      <span class="motor-badge ccw">CCW · GPIO27</span>
    </div>
    <div class="motor-value" id="v3">0 <span class="motor-pct">/ 255</span></div>
    <input type="range" min="0" max="180" value="0"
      oninput="send(3,this.value)" onchange="send(3,this.value)" id="s3">
    <button class="stop-btn" onclick="stopMotor(3)">Stop M3</button>
  </div>

  <div class="motor-card">
    <div class="motor-header">
      <span class="motor-name">M4 — Front-left</span>
      <span class="motor-badge ccw">CCW · GPIO14</span>
    </div>
    <div class="motor-value" id="v4">0 <span class="motor-pct">/ 255</span></div>
    <input type="range" min="0" max="180" value="0"
      oninput="send(4,this.value)" onchange="send(4,this.value)" id="s4">
    <button class="stop-btn" onclick="stopMotor(4)">Stop M4</button>
  </div>

  <button class="stop-all" onclick="stopAll()">⛔ STOP ALL MOTORS</button>
  <div class="status" id="status"></div>

  <script>
    function send(motor, value) {
      document.getElementById('v'+motor).innerHTML =
        value + ' <span class="motor-pct">/ 255 (' +
        Math.round(value/180*100) + '%)</span>';
      fetch('/motor?m='+motor+'&v='+value)
        .then(()=>{ document.getElementById('status').textContent = 'M'+motor+' → '+value; })
        .catch(()=>{ document.getElementById('status').textContent = 'Connection lost'; });
    }
    function stopMotor(m) {
      document.getElementById('s'+m).value = 0;
      send(m, 0);
    }
    function stopAll() {
      for(let i=1;i<=4;i++) stopMotor(i);
      fetch('/stop');
      document.getElementById('status').textContent = 'All motors stopped';
    }
    // Safety: stop all if page is closed or navigated away
    window.addEventListener('beforeunload', ()=>{ fetch('/stop'); });
  </script>
</body>
</html>
)rawhtml";

// ── HTTP handlers ─────────────────────────────────────────
void handleRoot() {
  server.send(200, "text/html", HTML);
}

void handleMotor() {
  if (!server.hasArg("m") || !server.hasArg("v")) {
    server.send(400, "text/plain", "Missing args");
    return;
  }
  int motor = server.arg("m").toInt();
  int value = server.arg("v").toInt();

  // channel = motor - 1 (motor 1 = channel 0, etc.)
  if (motor >= 1 && motor <= 4) {
    setMotor(motor - 1, value);  // motor 1 → index 0, etc.
    Serial.printf("M%d → %d\n", motor, value);
  }
  server.send(200, "text/plain", "OK");
}

void handleStop() {
  stopAllMotors();
  server.send(200, "text/plain", "Stopped");
  Serial.println("All motors stopped");
}

// ── Setup ─────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n\nDrone Motor Test — Phase 1");

  // Init all 4 motor PWM channels (ESP32 Arduino core 3.x API)
  int pins[4] = {M1_PIN, M2_PIN, M3_PIN, M4_PIN};
  for (int i = 0; i < 4; i++) {
    ledcAttach(pins[i], PWM_FREQ, PWM_RES);
    ledcWrite(pins[i], 0);
    Serial.printf("Motor %d → GPIO%d\n", i+1, pins[i]);
  }

  // Connect to WiFi with static IP
  WiFi.config(STATIC_IP, GATEWAY, SUBNET);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setHostname("esp32-drone");

  Serial.printf("Connecting to %s", WIFI_SSID);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    attempts++;
    if (attempts > 30) {
      Serial.println("\nWiFi failed — check credentials");
      // Restart and try again
      ESP.restart();
    }
  }

  Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());
  Serial.println("Open this address in your browser");

  // Register HTTP routes
  server.on("/",      handleRoot);
  server.on("/motor", handleMotor);
  server.on("/stop",  handleStop);
  server.begin();
  Serial.println("Web server started");
}

// ── Loop ──────────────────────────────────────────────────
void loop() {
  server.handleClient();

  // Safety watchdog: if WiFi drops, stop motors immediately
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 3000) {
    lastCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi lost — stopping motors");
      stopAllMotors();
    }
  }
}
