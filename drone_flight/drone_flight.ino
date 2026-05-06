/*
 * DRONE FLIGHT FIRMWARE — Phase 3
 * ─────────────────────────────────────────────────────────
 * Full PID stabilisation + WebSocket control
 *
 * MOTOR PINS:
 *   M1 Front-right CW  → GPIO25
 *   M2 Rear-left   CW  → GPIO26
 *   M3 Rear-right  CCW → GPIO27
 *   M4 Front-left  CCW → GPIO14
 *
 * IMU:
 *   SDA → GPIO21  SCL → GPIO22  INT → GPIO4
 *
 * CONTROL (from Android PWA):
 *   Left joystick  → Throttle (up/down) + Yaw (left/right)
 *   Right joystick → Pitch (fwd/back)   + Roll (left/right)
 *   Or tilt phone for pitch/roll
 * ─────────────────────────────────────────────────────────
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ArduinoJson.h>

// ── WiFi ──────────────────────────────────────────────────
const char* WIFI_SSID     = "your-wifi-ssid";       // ← change this
const char* WIFI_PASSWORD = "your-wifi-password";   // ← change this

// ── Static IP config ──────────────────────────────────────
// Change last number to any free address on your network
// e.g. if your phone shows 192.168.1.5, use 192.168.1.100
IPAddress STATIC_IP(192, 168, -, -);   // ← change to match your network
IPAddress GATEWAY  (192, 168, -, -);     // ← usually your router IP
IPAddress SUBNET   (255, 255, 255, 0);

// ── Motor pins ────────────────────────────────────────────
#define M1_PIN  25
#define M2_PIN  26
#define M3_PIN  27
#define M4_PIN  14
int MOTOR_PINS[4] = {M1_PIN, M2_PIN, M3_PIN, M4_PIN};

// ── PWM ───────────────────────────────────────────────────
#define PWM_FREQ      25000
#define PWM_RES       8
#define MIN_THROTTLE  20    // minimum PWM to spin motors
#define MAX_THROTTLE  200   // maximum PWM (leave headroom)
#define THROTTLE_CAP  160   // safety cap for first flights

// ── IMU ───────────────────────────────────────────────────
#define SDA_PIN 21
#define SCL_PIN 22
#define INVERT_X false
#define INVERT_Y false
#define INVERT_Z false
#define ALPHA 0.96f

// ── PID gains — START CONSERVATIVE, tune up slowly ───────
// Roll
float Kp_roll  = 1.0f, Ki_roll  = 0.01f, Kd_roll  = 0.3f;
// Pitch
float Kp_pitch = 1.0f, Ki_pitch = 0.01f, Kd_pitch = 0.3f;
// Yaw (rate-based)
float Kp_yaw   = 2.0f, Ki_yaw   = 0.0f,  Kd_yaw   = 0.1f;

// ── Safety ────────────────────────────────────────────────
#define WATCHDOG_MS       500   // stop motors if no command received
#define MAX_ANGLE_DEG     45.0f // kill motors if tilted beyond this
#define PID_CLAMP         80.0f // max PID output per axis

// ── Objects ───────────────────────────────────────────────
MPU6050 mpu;
WebServer server(80);
WebSocketsServer webSocket(81);

// ── IMU state ─────────────────────────────────────────────
float roll = 0, pitch = 0, yaw = 0;
float gyroX_off = 0, gyroY_off = 0, gyroZ_off = 0;
float accelX_off = 0, accelY_off = 0;
unsigned long lastIMUTime = 0;

// ── PID state ─────────────────────────────────────────────
float roll_err_prev  = 0, pitch_err_prev  = 0, yaw_err_prev  = 0;
float roll_integral  = 0, pitch_integral  = 0, yaw_integral  = 0;

// ── Control setpoints (from app) ─────────────────────────
volatile float sp_throttle = 0;  // 0–100 mapped to PWM
volatile float sp_roll     = 0;  // degrees
volatile float sp_pitch    = 0;  // degrees
volatile float sp_yaw      = 0;  // deg/s rate
volatile bool  armed       = false;
unsigned long  lastCmdTime = 0;

// ── Motor outputs ─────────────────────────────────────────
int m1_out = 0, m2_out = 0, m3_out = 0, m4_out = 0;

// ── Calibration ───────────────────────────────────────────
void calibrateIMU() {
  Serial.println("Calibrating — keep drone STILL...");
  long gx=0,gy=0,gz=0,ax=0,ay=0;
  const int N = 500;
  for (int i = 0; i < N; i++) {
    int16_t ax_,ay_,az_,gx_,gy_,gz_;
    mpu.getMotion6(&ax_,&ay_,&az_,&gx_,&gy_,&gz_);
    gx+=gx_; gy+=gy_; gz+=gz_;
    ax+=ax_; ay+=ay_;
    delay(2);
  }
  gyroX_off  = gx/(float)N/131.0f;
  gyroY_off  = gy/(float)N/131.0f;
  gyroZ_off  = gz/(float)N/131.0f;
  accelX_off = ax/(float)N/16384.0f;
  accelY_off = ay/(float)N/16384.0f;
  Serial.println("Calibration done.");
}

// ── IMU update ────────────────────────────────────────────
float raw_gx=0, raw_gy=0, raw_gz=0;
float raw_ax=0, raw_ay=0, raw_az=0;

void updateIMU() {
  unsigned long now = micros();
  float dt = (now - lastIMUTime) / 1000000.0f;
  lastIMUTime = now;
  if (dt < 0.0001f || dt > 0.1f) return;

  int16_t ax,ay,az,gx,gy,gz;
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

  float aX = ax/16384.0f - accelX_off;
  float aY = ay/16384.0f - accelY_off;
  float aZ = az/16384.0f;
  float gX = gx/131.0f   - gyroX_off;
  float gY = gy/131.0f   - gyroY_off;
  float gZ = gz/131.0f   - gyroZ_off;

  if (INVERT_X) { aX=-aX; gX=-gX; }
  if (INVERT_Y) { aY=-aY; gY=-gY; }
  if (INVERT_Z) { gZ=-gZ; }

  raw_ax=aX; raw_ay=aY; raw_az=aZ;
  raw_gx=gX; raw_gy=gY; raw_gz=gZ;

  float rollAcc  = atan2(aY, aZ) * 180.0f / PI;
  float pitchAcc = atan2(-aX, sqrt(aY*aY+aZ*aZ)) * 180.0f / PI;

  roll  = ALPHA*(roll  + gX*dt) + (1.0f-ALPHA)*rollAcc;
  pitch = ALPHA*(pitch + gY*dt) + (1.0f-ALPHA)*pitchAcc;
  yaw  += gZ*dt;

  if (yaw >  180.0f) yaw -= 360.0f;
  if (yaw < -180.0f) yaw += 360.0f;
}

// ── PID ───────────────────────────────────────────────────
float pidCalc(float setpoint, float measured, float gyroRate,
              float Kp, float Ki, float Kd,
              float &integral, float &prev_err, float dt) {
  float err = setpoint - measured;
  integral += err * dt;
  integral = constrain(integral, -50.0f, 50.0f); // anti-windup
  float derivative = -gyroRate; // use gyro for derivative (smoother)
  float output = Kp*err + Ki*integral + Kd*derivative;
  prev_err = err;
  return constrain(output, -PID_CLAMP, PID_CLAMP);
}

// ── Motor write ───────────────────────────────────────────
void writeMotor(int pin, int value) {
  value = constrain(value, 0, THROTTLE_CAP);
  ledcWrite(pin, value);
}

void stopAllMotors() {
  for (int i=0; i<4; i++) ledcWrite(MOTOR_PINS[i], 0);
  m1_out=m2_out=m3_out=m4_out=0;
}

// ── Flight loop ───────────────────────────────────────────
void flightLoop(float dt) {
  // Safety checks
  bool watchdogOK = (millis() - lastCmdTime) < WATCHDOG_MS;
  bool angleOK    = (abs(roll) < MAX_ANGLE_DEG) && (abs(pitch) < MAX_ANGLE_DEG);

  if (!armed || !watchdogOK || !angleOK) {
    stopAllMotors();
    if (!armed)      {}
    if (!watchdogOK) Serial.println("WATCHDOG — no command received");
    if (!angleOK)    Serial.println("ANGLE LIMIT — motors killed");
    // Reset integrals on disarm
    roll_integral=pitch_integral=yaw_integral=0;
    return;
  }

  // Map throttle 0-100 → MIN_THROTTLE to MAX_THROTTLE
  int base = (int)map((long)sp_throttle, 0, 100, MIN_THROTTLE, MAX_THROTTLE);
  base = constrain(base, MIN_THROTTLE, THROTTLE_CAP);

  // PID outputs
  float roll_out  = pidCalc(sp_roll,  roll,  raw_gx,
                            Kp_roll,  Ki_roll,  Kd_roll,
                            roll_integral,  roll_err_prev,  dt);
  float pitch_out = pidCalc(sp_pitch, pitch, raw_gy,
                            Kp_pitch, Ki_pitch, Kd_pitch,
                            pitch_integral, pitch_err_prev, dt);
  float yaw_out   = pidCalc(sp_yaw,   raw_gz, 0,
                            Kp_yaw,   Ki_yaw,   Kd_yaw,
                            yaw_integral,   yaw_err_prev,   dt);

  // Motor mixing — standard X frame
  // M1=FR·CW   M2=RL·CW   M3=RR·CCW  M4=FL·CCW
  m1_out = base - pitch_out + roll_out - yaw_out;
  m2_out = base + pitch_out - roll_out - yaw_out;
  m3_out = base + pitch_out + roll_out + yaw_out;
  m4_out = base - pitch_out - roll_out + yaw_out;

  writeMotor(M1_PIN, m1_out);
  writeMotor(M2_PIN, m2_out);
  writeMotor(M3_PIN, m3_out);
  writeMotor(M4_PIN, m4_out);
}

// ── WebSocket handler ─────────────────────────────────────
void onWebSocket(uint8_t client, WStype_t type,
                 uint8_t* payload, size_t len) {
  if (type == WStype_TEXT) {
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, payload)) return;

    // Update setpoints
    if (doc.containsKey("t")) sp_throttle = constrain((float)doc["t"], 0, 100);
    if (doc.containsKey("r")) sp_roll     = constrain((float)doc["r"], -30, 30);
    if (doc.containsKey("p")) sp_pitch    = constrain((float)doc["p"], -30, 30);
    if (doc.containsKey("y")) sp_yaw      = constrain((float)doc["y"], -100, 100);
    if (doc.containsKey("arm")) {
      armed = (bool)doc["arm"];
      if (!armed) stopAllMotors();
      Serial.printf("Armed: %s\n", armed ? "YES" : "NO");
    }
    // PID tuning from app
    if (doc.containsKey("kp_r")) Kp_roll  = doc["kp_r"];
    if (doc.containsKey("ki_r")) Ki_roll  = doc["ki_r"];
    if (doc.containsKey("kd_r")) Kd_roll  = doc["kd_r"];
    if (doc.containsKey("kp_p")) Kp_pitch = doc["kp_p"];
    if (doc.containsKey("ki_p")) Ki_pitch = doc["ki_p"];
    if (doc.containsKey("kd_p")) Kd_pitch = doc["kd_p"];

    lastCmdTime = millis();
  } else if (type == WStype_DISCONNECTED) {
    Serial.println("App disconnected — stopping motors");
    armed = false;
    stopAllMotors();
  }
}

// ── App HTML (served at root) ─────────────────────────────
const char* APP_HTML = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<meta name="mobile-web-app-capable" content="yes">
<title>Drone Control</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent;user-select:none}
body{background:#0a0a0e;color:#e8e6de;font-family:-apple-system,sans-serif;
     height:100vh;overflow:hidden;display:flex;flex-direction:column}
#header{display:flex;align-items:center;justify-content:space-between;
        padding:10px 16px;background:#12121a;border-bottom:1px solid #1e1e2a;flex-shrink:0}
.logo{font-size:14px;font-weight:500}
.dot{width:8px;height:8px;border-radius:50%;background:#f87171;display:inline-block;margin-right:6px}
.dot.on{background:#4ade80}
#statusBar{font-size:11px;color:#888}
#telemetry{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;
           padding:8px 12px;background:#12121a;flex-shrink:0}
.tel-box{background:#1a1a22;border-radius:6px;padding:6px 8px;text-align:center}
.tel-label{font-size:9px;color:#666;text-transform:uppercase;letter-spacing:0.06em}
.tel-val{font-size:16px;font-weight:300;color:#fff;font-variant-numeric:tabular-nums}
#controls{flex:1;display:flex;flex-direction:column;padding:8px}
#joystick-area{flex:1;display:flex;align-items:center;justify-content:space-around;position:relative}
.js-container{width:140px;height:140px;position:relative}
.js-base{width:140px;height:140px;border-radius:50%;background:#1a1a22;
         border:2px solid #2a2a36;position:absolute;top:0;left:0}
.js-label{position:absolute;font-size:9px;color:#444;text-transform:uppercase}
.js-label.top{top:8px;left:50%;transform:translateX(-50%)}
.js-label.bottom{bottom:8px;left:50%;transform:translateX(-50%)}
.js-label.left{left:8px;top:50%;transform:translateY(-50%)}
.js-label.right{right:8px;top:50%;transform:translateY(-50%)}
.js-stick{width:50px;height:50px;border-radius:50%;background:#2d5a8e;
          position:absolute;transform:translate(-50%,-50%);
          box-shadow:0 0 20px #2d5a8e55;transition:background 0.2s}
.js-stick.active{background:#4f8ef7}
#center-panel{display:flex;flex-direction:column;align-items:center;gap:8px}
#arm-btn{width:72px;height:72px;border-radius:50%;border:none;
         background:#1a1a1a;border:2px solid #3a1a1a;color:#f87171;
         font-size:11px;font-weight:500;cursor:pointer;letter-spacing:0.05em}
#arm-btn.armed{background:#1a2a1a;border-color:#1a4a1a;color:#4ade80}
#stop-btn{width:56px;height:56px;border-radius:50%;border:none;
          background:#7f1d1d;color:#fff;font-size:10px;font-weight:600;cursor:pointer}
#tilt-btn{padding:6px 14px;border-radius:20px;border:1px solid #2a2a36;
          background:#1a1a22;color:#888;font-size:11px;cursor:pointer}
#tilt-btn.active{background:#1a2a3a;border-color:#2d5a8e;color:#60a5fa}
#bottom-bar{padding:8px 16px;background:#12121a;border-top:1px solid #1e1e2a;
            display:flex;align-items:center;gap:10px;flex-shrink:0}
.pid-row{display:flex;align-items:center;gap:6px;font-size:11px;color:#666}
.pid-row label{width:36px}
.pid-row input{width:56px;background:#1a1a22;border:1px solid #2a2a36;
               color:#e8e6de;border-radius:4px;padding:2px 6px;font-size:11px}
#pid-panel{display:none;padding:10px 12px;background:#12121a;
           border-top:1px solid #1e1e2a;flex-shrink:0}
#pid-toggle{padding:4px 10px;border-radius:4px;border:1px solid #2a2a36;
            background:#1a1a22;color:#888;font-size:11px;cursor:pointer}
#thr-display{font-size:11px;color:#f59e0b;min-width:40px;text-align:center}
</style>
</head>
<body>

<div id="header">
  <div class="logo">🚁 Drone Control</div>
  <div>
    <span class="dot" id="dot"></span>
    <span id="statusBar">Disconnected</span>
  </div>
</div>

<div id="telemetry">
  <div class="tel-box">
    <div class="tel-label">Roll</div>
    <div class="tel-val" id="t-roll">0.0°</div>
  </div>
  <div class="tel-box">
    <div class="tel-label">Pitch</div>
    <div class="tel-val" id="t-pitch">0.0°</div>
  </div>
  <div class="tel-box">
    <div class="tel-label">Yaw</div>
    <div class="tel-val" id="t-yaw">0.0°</div>
  </div>
  <div class="tel-box">
    <div class="tel-label">Throttle</div>
    <div class="tel-val" id="t-thr">0%</div>
  </div>
</div>

<div id="controls">
  <div id="joystick-area">

    <!-- Left joystick: Throttle + Yaw -->
    <div class="js-container" id="jsL-container">
      <div class="js-base">
        <span class="js-label top">THR+</span>
        <span class="js-label bottom">THR-</span>
        <span class="js-label left">YAW L</span>
        <span class="js-label right">YAW R</span>
      </div>
      <div class="js-stick" id="jsL-stick"></div>
    </div>

    <!-- Center panel -->
    <div id="center-panel">
      <button id="arm-btn" onclick="toggleArm()">DISARMED</button>
      <button id="stop-btn" onclick="emergencyStop()">STOP</button>
      <button id="tilt-btn" onclick="toggleTilt()">TILT OFF</button>
      <span id="thr-display">0%</span>
    </div>

    <!-- Right joystick: Pitch + Roll -->
    <div class="js-container" id="jsR-container">
      <div class="js-base">
        <span class="js-label top">PITCH+</span>
        <span class="js-label bottom">PITCH-</span>
        <span class="js-label left">ROLL L</span>
        <span class="js-label right">ROLL R</span>
      </div>
      <div class="js-stick" id="jsR-stick"></div>
    </div>

  </div>
</div>

<div id="bottom-bar">
  <button id="pid-toggle" onclick="togglePID()">PID ▾</button>
  <div style="font-size:11px;color:#555">M1:<span id="m1">0</span>
    M2:<span id="m2">0</span> M3:<span id="m3">0</span> M4:<span id="m4">0</span></div>
</div>

<div id="pid-panel">
  <div style="font-size:11px;color:#666;margin-bottom:8px">PID Tuning (changes apply live)</div>
  <div class="pid-row">
    <label>Kp_R</label><input id="kp_r" value="1.0" onchange="sendPID()">
    <label>Ki_R</label><input id="ki_r" value="0.01" onchange="sendPID()">
    <label>Kd_R</label><input id="kd_r" value="0.3" onchange="sendPID()">
  </div>
  <div class="pid-row" style="margin-top:6px">
    <label>Kp_P</label><input id="kp_p" value="1.0" onchange="sendPID()">
    <label>Ki_P</label><input id="ki_p" value="0.01" onchange="sendPID()">
    <label>Kd_P</label><input id="kd_p" value="0.3" onchange="sendPID()">
  </div>
</div>

<script>
// ── WebSocket ─────────────────────────────────────────────
let ws, armed=false, tiltMode=false;
let throttle=0, yaw=0, pitchSP=0, rollSP=0;

function connect() {
  ws = new WebSocket('ws://'+location.hostname+':81');
  ws.onopen = () => {
    document.getElementById('dot').className='dot on';
    document.getElementById('statusBar').textContent='Connected';
  };
  ws.onclose = () => {
    document.getElementById('dot').className='dot';
    document.getElementById('statusBar').textContent='Disconnected — reconnecting...';
    armed=false; updateArmBtn();
    setTimeout(connect, 2000);
  };
  ws.onmessage = (e) => {
    const d = JSON.parse(e.data);
    document.getElementById('t-roll').textContent  = d.roll?.toFixed(1)+'°';
    document.getElementById('t-pitch').textContent = d.pitch?.toFixed(1)+'°';
    document.getElementById('t-yaw').textContent   = d.yaw?.toFixed(1)+'°';
    document.getElementById('t-thr').textContent   = Math.round(throttle)+'%';
    if(d.m1!==undefined){
      document.getElementById('m1').textContent=d.m1;
      document.getElementById('m2').textContent=d.m2;
      document.getElementById('m3').textContent=d.m3;
      document.getElementById('m4').textContent=d.m4;
    }
  };
}
connect();

function send(obj) {
  if(ws && ws.readyState===1) ws.send(JSON.stringify(obj));
}

// ── Arm / stop ────────────────────────────────────────────
function toggleArm() {
  if(!armed && throttle > 5) {
    alert('Lower throttle to 0 before arming'); return;
  }
  armed = !armed;
  send({arm: armed});
  updateArmBtn();
}
function updateArmBtn() {
  const b = document.getElementById('arm-btn');
  b.textContent = armed ? 'ARMED' : 'DISARMED';
  b.className = armed ? 'armed' : '';
}
function emergencyStop() {
  armed=false; throttle=0; pitchSP=0; rollSP=0; yaw=0;
  send({arm:false, t:0, p:0, r:0, y:0});
  updateArmBtn();
  resetJoysticks();
}

// ── Tilt control ──────────────────────────────────────────
function toggleTilt() {
  tiltMode = !tiltMode;
  const b = document.getElementById('tilt-btn');
  b.textContent = tiltMode ? 'TILT ON' : 'TILT OFF';
  b.className   = tiltMode ? 'active' : '';
  if(tiltMode) {
    window.addEventListener('deviceorientation', onTilt);
  } else {
    window.removeEventListener('deviceorientation', onTilt);
    pitchSP=0; rollSP=0;
  }
}
function onTilt(e) {
  // gamma = left/right tilt = roll, beta = fwd/back = pitch
  rollSP  = constrain(e.gamma * 0.6, -30, 30);
  pitchSP = constrain(e.beta  * 0.6, -30, 30);
}
function constrain(v,mn,mx){return Math.min(mx,Math.max(mn,v));}

// ── Control loop — send at 50Hz ───────────────────────────
setInterval(()=>{
  if(!ws||ws.readyState!==1) return;
  send({t:throttle, r:rollSP, p:pitchSP, y:yaw});
  document.getElementById('thr-display').textContent=Math.round(throttle)+'%';
}, 20);

// ── PID panel ─────────────────────────────────────────────
function togglePID() {
  const p = document.getElementById('pid-panel');
  p.style.display = p.style.display==='none'?'block':'none';
}
function sendPID() {
  send({
    kp_r: parseFloat(document.getElementById('kp_r').value),
    ki_r: parseFloat(document.getElementById('ki_r').value),
    kd_r: parseFloat(document.getElementById('kd_r').value),
    kp_p: parseFloat(document.getElementById('kp_p').value),
    ki_p: parseFloat(document.getElementById('ki_p').value),
    kd_p: parseFloat(document.getElementById('kd_p').value),
  });
}

// ── Joystick implementation ───────────────────────────────
function makeJoystick(containerId, stickId, onMove) {
  const container = document.getElementById(containerId);
  const stick     = document.getElementById(stickId);
  const R = 70; // radius of base
  const SR = 25; // stick radius
  let active=false, cx=R, cy=R;

  // Position stick at center initially
  stick.style.left = R+'px';
  stick.style.top  = R+'px';

  function getPos(e) {
    const rect = container.getBoundingClientRect();
    const touch = e.touches ? e.touches[0] : e;
    return {
      x: touch.clientX - rect.left,
      y: touch.clientY - rect.top
    };
  }

  function moveStick(pos) {
    let dx = pos.x - R;
    let dy = pos.y - R;
    const dist = Math.sqrt(dx*dx+dy*dy);
    if(dist > R-SR) {
      dx = dx/dist*(R-SR);
      dy = dy/dist*(R-SR);
    }
    stick.style.left = (R+dx)+'px';
    stick.style.top  = (R+dy)+'px';
    // Normalize to -1..1
    onMove(dx/(R-SR), dy/(R-SR));
  }

  container.addEventListener('touchstart', e=>{
    e.preventDefault(); active=true;
    stick.className='js-stick active';
    moveStick(getPos(e));
  },{passive:false});

  container.addEventListener('touchmove', e=>{
    e.preventDefault();
    if(active) moveStick(getPos(e));
  },{passive:false});

  container.addEventListener('touchend', e=>{
    e.preventDefault(); active=false;
    stick.className='js-stick';
    stick.style.left=R+'px'; stick.style.top=R+'px';
    onMove(0,0);
  },{passive:false});

  // Mouse support for desktop testing
  container.addEventListener('mousedown', e=>{
    active=true; stick.className='js-stick active';
    moveStick(getPos(e));
  });
  document.addEventListener('mousemove', e=>{
    if(active) moveStick(getPos(e));
  });
  document.addEventListener('mouseup', e=>{
    if(active){
      active=false; stick.className='js-stick';
      stick.style.left=R+'px'; stick.style.top=R+'px';
      onMove(0,0);
    }
  });
}

function resetJoysticks() {
  ['jsL-stick','jsR-stick'].forEach(id=>{
    const s=document.getElementById(id);
    s.style.left='70px'; s.style.top='70px';
  });
  throttle=0; yaw=0; pitchSP=0; rollSP=0;
}

// Left joystick: up/down = throttle, left/right = yaw
makeJoystick('jsL-container','jsL-stick',(x,y)=>{
  // Y is inverted (up = negative y in screen coords)
  throttle = constrain(throttle + (-y * 2), 0, 100);
  yaw = x * 100;
});

// Right joystick: up/down = pitch, left/right = roll
makeJoystick('jsR-container','jsR-stick',(x,y)=>{
  if(!tiltMode) {
    pitchSP = -y * 30;  // up = forward = positive pitch
    rollSP  =  x * 30;  // right = positive roll
  }
});
</script>
</body>
</html>
)rawhtml";

// ── HTTP ──────────────────────────────────────────────────
void handleRoot() { server.send(200,"text/html",APP_HTML); }

// ── Setup ─────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\nDrone Flight Firmware — Phase 3");

  // Motors
  for (int i=0; i<4; i++) {
    ledcAttach(MOTOR_PINS[i], PWM_FREQ, PWM_RES);
    ledcWrite(MOTOR_PINS[i], 0);
  }
  Serial.println("Motors initialised");

  // IMU
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 FAIL — check wiring");
    while(1) delay(500);
  }
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  Serial.println("MPU6050 OK");

  // WiFi
  WiFi.config(STATIC_IP, GATEWAY, SUBNET);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setHostname("esp32-drone");
  Serial.print("WiFi connecting");
  while(WiFi.status()!=WL_CONNECTED){delay(500);Serial.print(".");}
  Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());

  // Servers
  server.on("/", handleRoot);
  server.begin();
  webSocket.begin();
  webSocket.onEvent(onWebSocket);

  // Calibrate
  delay(500);
  calibrateIMU();
  lastIMUTime = micros();
  lastCmdTime = millis();

  Serial.println("Ready — open http://192.168.29.150 on your phone");
}

// ── Loop ──────────────────────────────────────────────────
unsigned long lastFlightLoop = 0;
unsigned long lastTelemetry  = 0;

void loop() {
  server.handleClient();
  webSocket.loop();

  unsigned long now = micros();
  float dt = (now - lastFlightLoop) / 1000000.0f;

  // PID loop at 100Hz
  if (dt >= 0.01f) {
    updateIMU();
    flightLoop(dt);
    lastFlightLoop = now;
  }

  // Telemetry to app at 20Hz
  if (millis() - lastTelemetry > 50) {
    lastTelemetry = millis();
    StaticJsonDocument<128> doc;
    doc["roll"]  = round(roll*10)/10.0;
    doc["pitch"] = round(pitch*10)/10.0;
    doc["yaw"]   = round(yaw*10)/10.0;
    doc["m1"]    = m1_out;
    doc["m2"]    = m2_out;
    doc["m3"]    = m3_out;
    doc["m4"]    = m4_out;
    String json; serializeJson(doc, json);
    webSocket.broadcastTXT(json);
  }
}
