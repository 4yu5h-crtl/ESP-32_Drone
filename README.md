# ESP-32 Drone 🚁

> A WiFi-controlled quadcopter built from scratch using an ESP32 microcontroller, MPU-6050 IMU, and custom brushed motor drivers — controlled from a browser-based app on your phone. No proprietary flight controller. No app installation. Just components, code, and physics.

---

## Demo

> *(Add your flight video/gif here)*

---

## What This Is

This project builds a fully functional drone using:
- An **ESP32-S** as the flight controller and WiFi access point
- An **MPU-6050** gyroscope + accelerometer for stabilisation
- **4 custom SI2300 MOSFET motor drivers** (one per brushed motor)
- A **browser-based PWA** served directly from the ESP32 — open it on any phone, no install needed
- A **PID stabilisation loop** running at 100Hz with live tuning from the app
- **WebSocket** for real-time bidirectional control + telemetry

Built and tested in Pune, India. Total component cost under ₹2000.

---

## Hardware

| Component | Spec | Pin(s) |
|---|---|---|
| ESP32-S dev board | 38-pin, HW-573 V1.3.1, CP2102 | — |
| MPU-6050 | 6-axis IMU, I2C, 3.3V | SDA→21, SCL→22, INT→4 |
| Motor M1 (Front-right) | Brushed DC, CW | GPIO25 |
| Motor M2 (Rear-left) | Brushed DC, CW | GPIO26 |
| Motor M3 (Rear-right) | Brushed DC, CCW | GPIO27 |
| Motor M4 (Front-left) | Brushed DC, CCW | GPIO14 |
| LiPo battery | 1S, 3.7V, 500mAh | 3V3 pin + motor VCC |
| Motor driver | Custom SI2300 N-MOSFET + 10kΩ gate resistor | PWM/VCC/GND |

### Wiring summary

```
LiPo (+) ──► Motor driver VCC × 4
         └──► ESP32 3V3 pin
LiPo (−) ──► Common GND (ESP32 + all 4 drivers)

ESP32 GPIO21 ──► MPU-6050 SDA
ESP32 GPIO22 ──► MPU-6050 SCL
ESP32 GPIO4  ──► MPU-6050 INT
ESP32 3V3    ──► MPU-6050 VCC
ESP32 GND    ──► MPU-6050 GND + MPU AD0

ESP32 GPIO25 ──► Driver M1 PWM
ESP32 GPIO26 ──► Driver M2 PWM
ESP32 GPIO27 ──► Driver M3 PWM
ESP32 GPIO14 ──► Driver M4 PWM
```

> ⚠️ Never connect LiPo to 3V3 while USB is also connected. Two power sources on the same rail will damage the voltage regulator.

---

## Repository Structure

```
ESP-32_Drone/
├── motor_test/
│   └── motor_test.ino       # Phase 1 — browser-based individual motor testing
├── imu_test/
│   └── imu_test.ino         # Phase 2 — live IMU visualisation + axis verification
└── drone_flight/
    └── drone_flight.ino     # Phase 3 — full PID flight firmware + control app
```

---

## Setup

### Requirements

- Arduino IDE 2.x
- ESP32 board package by Espressif (v3.x)
- Libraries (install via Library Manager):
  - `MPU6050` by Electronic Cats
  - `WebSockets` by Markus Sattler
  - `ArduinoJson` by Benoit Blanchon

### Board settings

| Setting | Value |
|---|---|
| Board | ESP32 Dev Module |
| Upload Speed | 921600 |
| CPU Frequency | 240MHz |
| Flash Size | 4MB |
| Partition Scheme | Default 4MB with spiffs |

---

## Phase 1 — Motor Test

**File:** `motor_test/motor_test.ino`

Tests each motor independently via a browser interface. Use this to verify all 4 motors respond and spin in the correct direction.

### Configure before flashing

Open `motor_test.ino` and edit lines 22–28:

```cpp
const char* WIFI_SSID     = "YOUR_WIFI_NAME";      // ← your home WiFi SSID
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";  // ← your WiFi password

IPAddress STATIC_IP(192, 168, 29, 150);  // ← match your network
IPAddress GATEWAY  (192, 168, 29, 1);   // ← your router IP
IPAddress SUBNET   (255, 255, 255, 0);
```

> To find your network range: on your phone → WiFi settings → tap your network → note the IP shown (e.g. `192.168.29.8`). Use the same first three numbers for STATIC_IP and GATEWAY.

### Flash and run

```
1. Select board: ESP32 Dev Module
2. Flash motor_test.ino
3. Open Serial Monitor at 115200 baud
4. Wait for: "Connected! IP: 192.168.29.150"
5. Open that IP in Chrome on your phone (same WiFi)
6. Use sliders to test each motor
```

### ⚠️ Remove all propellers before this test

### What to check

| Motor | GPIO | Expected direction |
|---|---|---|
| M1 Front-right | 25 | CW (clockwise) |
| M2 Rear-left | 26 | CW |
| M3 Rear-right | 27 | CCW (counter-clockwise) |
| M4 Front-left | 14 | CCW |

Wrong direction = swap that motor's two power wires at the driver board. No code change needed.

---

## Phase 2 — IMU Test

**File:** `imu_test/imu_test.ino`

Streams live roll/pitch/yaw to browser via WebSocket. Use this to verify IMU axes are correct before any flight attempt.

### Configure before flashing

Same WiFi + IP block as Phase 1 (lines 22–28). Also check:

```cpp
// Line 35-37 — axis inversion for your MPU mount orientation
#define INVERT_X  false   // flip if roll goes wrong direction
#define INVERT_Y  false   // flip if pitch goes wrong direction
#define INVERT_Z  false   // flip if yaw goes wrong direction
```

### Flash and run

```
1. Flash imu_test.ino
2. Keep drone STILL for 1 second after power-on (gyro calibration)
3. Open 192.168.29.150 in Chrome
4. Run axis verification:
   - Lay flat → Roll ≈ 0°, Pitch ≈ 0°
   - Tilt nose down → Pitch goes POSITIVE ✓
   - Tilt right side down → Roll goes POSITIVE ✓
   - Rotate CW from above → Yaw goes POSITIVE ✓
```

If any axis is inverted, toggle the corresponding `INVERT_` flag and reflash.

---

## Phase 3 — Full Flight Firmware

**File:** `drone_flight/drone_flight.ino`

Full PID stabilisation loop + WebSocket control app served directly from ESP32. Open the IP on your phone — no app download needed.

### Configure before flashing

**Step 1 — WiFi credentials (lines 27–28):**
```cpp
const char* WIFI_SSID     = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
```

**Step 2 — Static IP (lines 31–33):**
```cpp
IPAddress STATIC_IP(192, 168, 29, 150);
IPAddress GATEWAY  (192, 168, 29, 1);
IPAddress SUBNET   (255, 255, 255, 0);
```

**Step 3 — Safety throttle cap (line 53):**
```cpp
#define THROTTLE_CAP  160   // 0-255. Keep at 160 for first flights.
                            // Raise to 200 only after stable hover confirmed.
```

**Step 4 — PID gains (lines 57–62):**
```cpp
float Kp_roll  = 1.0f, Ki_roll  = 0.01f, Kd_roll  = 0.3f;
float Kp_pitch = 1.0f, Ki_pitch = 0.01f, Kd_pitch = 0.3f;
float Kp_yaw   = 2.0f, Ki_yaw   = 0.0f,  Kd_yaw   = 0.1f;
```
These are conservative starting values. Use the in-app PID panel to tune live without reflashing.

### Flash and run

```
1. Flash drone_flight.ino
2. Open Serial Monitor at 115200
3. Wait for: "Ready — open http://192.168.29.150 on your phone"
4. Connect phone to your home WiFi
5. Open 192.168.29.150 in Chrome
6. Control app loads automatically — no install needed
```

### Control app layout

```
┌─────────────────────────────────────────┐
│  🚁 Drone Control          ● Connected  │
├──────────┬──────────┬──────────┬────────┤
│  Roll    │  Pitch   │   Yaw    │  Thr   │
├──────────┴──────────┴──────────┴────────┤
│                                          │
│  [Left joystick]  [ARM] [STOP] [TILT]   │
│  THR / YAW        [Right joystick]       │
│                   PITCH / ROLL           │
│                                          │
├─────────────────────────────────────────┤
│  [PID ▾]   M1:0  M2:0  M3:0  M4:0      │
└─────────────────────────────────────────┘
```

- **Left joystick:** up/down = throttle, left/right = yaw
- **Right joystick:** up/down = pitch, left/right = roll
- **TILT button:** switches right joystick to phone tilt control
- **ARM button:** must be green before motors respond
- **STOP button:** emergency cut — disarms and zeroes all setpoints
- **PID panel:** live tuning without reflash

### Arming procedure

```
1. Place drone on flat surface
2. Power on — wait 8 seconds for gyro calibration
3. Throttle at zero (left joystick fully down)
4. Tap DISARMED → button turns green (ARMED)
5. Slowly raise throttle
```

### Safety behaviour

| Condition | Action |
|---|---|
| App disconnects | Motors stop immediately |
| No command received for 500ms | Motors stop (watchdog) |
| Roll or pitch exceeds 45° | Motors stop (angle limit) |
| Throttle at zero + disarm | All motors stop, PID integrals reset |

---

## PID Tuning Guide

Start with defaults. Use the in-app PID panel to adjust live.

**Step 1 — Find Kp**
Set Ki=0, Kd=0. Raise Kp from 0.5 in steps of 0.2 until oscillation starts. Set Kp to 75% of that value.

**Step 2 — Add Kd**
Raise Kd from 0 in steps of 0.1. Reduces oscillation. Stop before motors get jittery.

**Step 3 — Add Ki**
Raise Ki from 0 in steps of 0.005. Corrects slow drift. Back off if slow oscillation develops.

---

## Troubleshooting

| Problem | Likely cause | Fix |
|---|---|---|
| `MPU6050 NOT detected` | SDA/SCL swapped or VCC wrong | Check GPIO21=SDA, GPIO22=SCL, 3.3V on VCC |
| Can't reach IP in browser | Wrong network or static IP conflict | Check Serial Monitor for actual IP, confirm phone is on same WiFi |
| Motors don't respond | Not armed, or throttle > 0 when arming | Lower throttle to 0, tap ARM button |
| Drone flips on takeoff | Wrong motor direction | Swap power wires on that motor |
| Drone oscillates violently | Kp too high | Reduce Kp_roll and Kp_pitch to 0.5 |
| Drone drifts in one direction | Needs Ki or trim | Increase Ki slightly (try 0.02) |
| WiFi not connecting | Wrong SSID/password or 5GHz network | ESP32 supports 2.4GHz only |

---

## How It Works

### Complementary filter
Blends gyroscope (fast, drifts) with accelerometer (slow, stable) for drift-free angle estimation:
```
angle = 0.96 × (angle + gyro × dt) + 0.04 × accel_angle
```

### PID loop
Runs at 100Hz. For each axis:
```
error      = setpoint − measured_angle
output     = Kp×error + Ki×∫error + Kd×(−gyro_rate)
```
Derivative uses raw gyro rate ("derivative on measurement") to avoid spikes on setpoint changes.

### Motor mixing (standard X-frame)
```
M1 (FR·CW)  = throttle − pitch + roll − yaw
M2 (RL·CW)  = throttle + pitch − roll − yaw
M3 (RR·CCW) = throttle + pitch + roll + yaw
M4 (FL·CCW) = throttle − pitch − roll + yaw
```

---

## Full Build Guide

Step-by-step build guide with diagrams, photos, and explanations:
- **Instructables:** *(add your link)*
- **Medium:** *(add your link)*

---

## Author

**Ayush Chintalwar**
Final-year CSE student · MIT ADT University, Pune
GitHub: [@4yu5h-crtl](https://github.com/4yu5h-crtl)

---

## License

MIT License — use freely, credit appreciated.
