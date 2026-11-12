# 🤖 EchoBot — AI-Powered Autonomous Trash Segregation Robot

<div align="center">

<video src="https://raw.githubusercontent.com/Mr-Patrick-James/echo_bot/main/ECOBOT%20PITCH%20DECK.mp4" autoplay loop muted playsinline width="100%"></video>

[![EchoBot Demo](https://img.shields.io/badge/▶%20Watch%20Demo-EchoBot%20POV-red?style=for-the-badge)](https://github.com/Mr-Patrick-James/echo_bot/blob/main/ECOBOT%20PITCH%20DECK.mp4)

> *EchoBot's point-of-view — autonomous trash detection and collection in action*

[![Edge Impulse](https://img.shields.io/badge/Edge%20Impulse-FOMO%20MobileNetV2-blueviolet)](https://edgeimpulse.com)
[![F1 Score](https://img.shields.io/badge/F1%20Score-88.4%25-brightgreen)](https://edgeimpulse.com)
[![ESP32](https://img.shields.io/badge/Platform-ESP32-blue)](https://www.espressif.com)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

</div>

---

## 📖 Overview

**EchoBot** is an AI-powered autonomous trash segregation robot that uses Edge Impulse FOMO object detection running on an ESP32-CAM to identify and classify waste into biodegradable, non-biodegradable, and recyclable categories in real time. It features a fully automated collection cycle — from detecting trash, navigating toward it, lifting, and segregating — all controlled through a custom WiFi web interface accessible from any device on the network. The system features a custom mobile-responsive web interface hosted directly on the ESP32, accessible via any browser on the same network, with five tabs — Car, Trash, All, Auto, and Dash — each dedicated to a specific control function ranging from manual driving and trash segregation to full automation and real-time system monitoring.

---

## ✨ Features

- 🎯 **AI Object Detection** — FOMO MobileNetV2 0.35 via Edge Impulse, 88.4% F1 Score
- 🚗 **Omni-Drive Movement** — 4-wheel drive with gear control (1-4) and joystick
- ♻️ **Auto Segregation** — Bio, Non-Bio, Recyclable with servo and conveyor
- 🤖 **Full Auto Mode** — Complete autonomous cycle from detection to segregation
- 📱 **Mobile Web UI** — 5-tab interface: Car, Trash, All, Auto, Dashboard
- 📡 **3-Board WiFi System** — ESP32 + ESP32-CAM + ESP8266 over HTTP
- ☀️ **Solar Powered** — 7x 18650 batteries + solar panel charging
- 🔊 **Melody Player** — Mario, Rick Roll, Imperial March and more
- 📊 **Live Dashboard** — Battery status, trash counters, system info

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    WiFi Network                          │
│                                                         │
│  ┌──────────────┐    HTTP     ┌──────────────────────┐  │
│  │ ESP32-CAM    │ ──────────► │ ESP32 Main Bot       │  │
│  │ 10.60.114.138│ /trackTrash │ 10.60.114.42         │  │
│  │              │ /trackLost  │                      │  │
│  │ FOMO Inference             │ Motors + Servos      │  │
│  │ MJPEG Stream │             │ Web Server (port 80) │  │
│  └──────────────┘             └──────────────────────┘  │
│                                        │                 │
│                               HTTP     │                 │
│                               ▼        │                 │
│                    ┌──────────────────┐│                 │
│                    │ ESP8266          ││                 │
│                    │ 10.60.114.130    ││                 │
│                    │ NEMA 17 Lifter   ││                 │
│                    └──────────────────┘│                 │
│                                        │                 │
│         📱 Browser ◄───────────────────┘                 │
│         (any device on network)                          │
└─────────────────────────────────────────────────────────┘
```

---

## 🔄 Full Auto Cycle

```
1. 📷 CAM detects trash (confidence ≥ 0.55)
         ↓
2. ⬇️  Lifter lowers to HOME (0 steps)
         ↓
3. ⚙️  Collector motor starts + Bot drives toward trash
         ↓
4. 📦  Trash enters collector → CAM loses sight → trackLost fires
         ↓
5. ⏹️  Bot stops + Collector stops
         ↓
6. ⬆️  Lifter goes UP (1580 steps)
         ↓
7. ⏳  Wait 2 seconds for trash to settle
         ↓
8. ♻️  Servo segregates by last detected label
         ↓
9. 🔄  CAM resumes → Ready for next trash
```

---

## 🧠 AI Model

| Property | Value |
|---|---|
| Model | FOMO (Faster Objects, More Objects) |
| Architecture | MobileNetV2 0.35 |
| Platform | Edge Impulse Studio |
| Inference Engine | EON™ Compiler (int8 quantized) |
| Input | 96×96 grayscale |
| Classes | bio, non_bio, recyclable |
| F1 Score | **88.4%** |
| Inference Time | 587ms |
| RAM Usage | 114.7KB |
| Training Dataset | 209 images |
| Training Trials | 10–20 experiments |

### Confusion Matrix

| | BIO | NON_BIO | RECYCLABLE |
|---|---|---|---|
| **Precision** | 81.2% | 100% | 90.5% |
| **F1** | 0.87 | 0.00 | 0.92 |

---

## 🛠️ Hardware Components

### Microcontrollers
| Component | Qty | Purpose |
|---|---|---|
| ESP32 Dev Module | 1 | Main controller |
| ESP32-CAM AI Thinker (OV3660) | 1 | AI inference + stream |
| ESP8266 NodeMCU | 1 | Trash lifter controller |

### Motors & Actuators
| Component | Qty | Purpose |
|---|---|---|
| TT DC Gear Motor | 6 | Drive + collector |
| NEMA 17 Stepper | 1 | Trash lifter |
| Standard Servo | 1 | Bin sorting arm |
| Continuous Rotation Servo | 1 | Conveyor belt |

### Drivers & Sensors
| Component | Qty | Purpose |
|---|---|---|
| L298N Dual H-Bridge | 2 | Motor control |
| A4988 Stepper Driver | 1 | NEMA 17 control |
| HC-SR04 Ultrasonic | 1 | Obstacle detection |
| Passive Buzzer | 1 | Audio feedback |

### Power System
| Component | Qty | Purpose |
|---|---|---|
| 18650 Battery (3200mAh) | 7 | Main power |
| 4-Slot Battery Holder | 1 | 14.8V for bot |
| 3-Slot Battery Holder | 1 | 11.1V for lifter |
| LM2596 Buck Converter | 1 | 14.8V → 5V |
| Solar Panel | 1 | Charging |

**Total Energy: ~82.9 Wh | Runtime: ~2-3 hours**

---

## 📌 Pin Configuration

### ESP32 Main Bot
| GPIO | Function |
|---|---|
| 26, 25 | Rear Left Motor (IN1, IN2) |
| 33, 32 | Rear Right Motor (IN3, IN4) |
| 18, 19 | Front Left Motor (IN5, IN6) |
| 4, 17 | Front Right Motor (IN7, IN8) |
| 15 | Sort Servo |
| 12 | Conveyor Servo |
| 27 | HC-SR04 TRIG |
| 14 | HC-SR04 ECHO |
| 23 | Buzzer |
| 21, 22 | Collector Motor (COL_A, COL_B) |

### ESP8266 Lifter
| Pin | Function |
|---|---|
| D1 (GPIO5) | A4988 STEP |
| D2 (GPIO4) | A4988 DIR |
| A0 | Battery voltage |

---

## 📡 HTTP API Routes

### ESP32 Main Bot (`10.60.114.42`)
| Route | Method | Description |
|---|---|---|
| `/` | GET | Web UI |
| `/joy?x=&y=` | GET | Joystick control |
| `/gear?g=` | GET | Speed gear 1-4 |
| `/MODE_MANUAL` | GET | Manual mode |
| `/MODE_AVOID` | GET | Obstacle avoid mode |
| `/MODE_FOLLOW` | GET | Follow mode |
| `/doBio` | GET | Servo → BIO |
| `/doNonBio` | GET | Servo → NON-BIO |
| `/doHome` | GET | Servo → HOME |
| `/doRecyclable` | GET | Conveyor cycle |
| `/collector?dir=&spd=` | GET | Collector motor |
| `/trackTrash` | GET | CAM detection trigger |
| `/trackLost` | GET | CAM lost trigger |
| `/camScan?state=` | GET | Toggle auto-sort |
| `/camMove?state=` | GET | Toggle auto-move |
| `/fullAuto?state=` | GET | Toggle full auto |
| `/cycleStatus` | GET | Cycle state JSON |
| `/dashboard` | GET | Dashboard data JSON |
| `/trashStatus` | GET | Servo status JSON |

### ESP8266 Lifter (`10.60.114.130`)
| Route | Method | Description |
|---|---|---|
| `/` | GET | Lifter UI |
| `/goHome` | GET | Move to 0 steps |
| `/goUp` | GET | Move to 1580 steps |
| `/moveTo?steps=` | GET | Move to exact steps |
| `/stop` | GET | Stop motor |
| `/start_cw` | GET | Continuous CW |
| `/start_ccw` | GET | Continuous CCW |
| `/setHome` | GET | Reset position to 0 |
| `/status` | GET | Position + battery JSON |
| `/battery` | GET | Battery voltage |

---

## 📚 Libraries

### ESP32 Main Bot
```cpp
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <HTTPClient.h>
```

### ESP32-CAM
```cpp
#include <esp_camera.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <object_detection_inferencing.h>  // Edge Impulse
```

### ESP8266
```cpp
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
```

---

## 🚀 Setup & Flashing

### Prerequisites
- Arduino IDE 2.x
- ESP32 board package
- ESP8266 board package
- Edge Impulse Arduino library (`ei-object_detection-arduino-1.0.1`)
- ESP32Servo library

### Flash Order
1. **Flash `echo_bot.ino`** first → confirm IP on Serial Monitor
2. **Flash `trash_lifter.ino`** → confirm IP on Serial Monitor
3. **Flash `CameraWebServer.ino`** → update `BOT_IP` in `ei_inference.h` first

### Configuration
Update WiFi credentials in each sketch:
```cpp
const char* ssid     = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
```

Update `BOT_IP` in `ei_inference.h`:
```cpp
#define BOT_IP "YOUR_BOT_IP"
```

Update lifter IP in `echo_bot.ino`:
```cpp
// Search for 10.60.114.130 and replace with your ESP8266 IP
```

---

## 🌐 Web Interface

Open `http://<BOT_IP>` in any browser on the same network.

| Tab | Description |
|---|---|
| 🚗 Car | Manual drive with joystick, modes, gears, songs |
| ♻️ Trash | Servo control, conveyor, CAM toggles, lifter |
| ⚡ All | All controls in one compressed landscape view |
| 🤖 Auto | Full auto mode with live CAM stream and cycle status |
| 📊 Dash | Trash counters, battery health, solar status, system info |

---

## 📊 Cost Breakdown

| Category | Estimated Cost |
|---|---|
| Microcontrollers (3 boards) | ~₱800 |
| Motors & Actuators | ~₱1,200 |
| Motor Drivers & Sensors | ~₱600 |
| Power System (batteries + solar) | ~₱1,500 |
| Miscellaneous (wires, breadboard, etc.) | ~₱715 |
| **Total** | **₱4,815** |

---

## 👥 Team

> EchoBot Prototype — AI-Powered Trash Segregation Robot
> Built with ❤️ using Edge Impulse, ESP32, and a lot of debugging

---

## 📄 License

MIT License — feel free to use, modify, and distribute with attribution.

---

<div align="center">
<sub>EchoBot | FOMO MobileNetV2 0.35 | F1: 88.4% | ESP32 + ESP32-CAM + ESP8266 | 7x 18650 | Solar Powered</sub>
</div>
