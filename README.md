# 📎 CTS-prevention-device

## Project Overview

Carpal Tunnel Syndrome (CTS) is a common repetitive strain injury caused by prolonged or improper wrist and finger movement, especially during tasks like typing or using digital devices. This project aims to design and build a **wearable system consisting of a wristband and a ring** to monitor hand and wrist activity patterns for **early detection and prevention of CTS**.

The system uses **IMU (Inertial Measurement Unit) sensors** placed on both the wrist and finger to detect:
- Repetitive finger movements
- Static or improper wrist angles
- Prolonged usage patterns

An **AI-based classifier** will later analyze collected data to detect CTS risk levels and notify the user. 
It includes integrating a mobile/web application for visualization. 
(And it may also include measuring handPWM using PPGs - Need further analysis.)

---

## 🧩 System Components

### 1. Wristband Module
- **IMU Sensor** (e.g., MPU6050): Tracks wrist orientation & motion
- **ESP32 MCU**: Processes data and handles Bluetooth communication
- **Vibration Motor**: Provides haptic feedback if posture is incorrect
- **Li-Po Battery + Charger**: Portable power supply

### 2. Ring Module
- **Mini IMU Sensor** (e.g., MPU6050 or MPU9250): Detects finger movement
- **Low-power MCU** (e.g., ESP32-C3 or nRF52832): Bluetooth communication
- **401012 LiPo Battery (Rechargeable)**: Lightweight and wearable

### 3. Communication
- **Bluetooth Low Energy (BLE)** between ring and wristband
- Optionally sends data to external app via BLE/Wi-Fi

### 4. AI & Application
- Data collected and labeled for AI model training (classification of risky usage)
- Optional mobile/web dashboard for:
  - Live posture monitoring
  - Movement heatmaps
  - Alert history and feedback

---

## 🧠 AI Model Scope (Phase II)
- Model Type: Decision Tree / SVM / LSTM (to be finalized)
- Input Features:
  - Wrist angle + duration
  - Finger motion frequency
  - Relative movement patterns
- Output: Risk score or posture alert
- Model Training: Local or cloud-based using collected labeled data
- Deployment: On-device (basic threshold logic) or app (model inference)

---

## Block diagram

<img width="669" height="346" alt="image" src="https://github.com/user-attachments/assets/b355d1b3-5fa0-47c1-9d79-e35c50f1208b" />




## 📅 Project Timeline (3-Month Plan)

| **Week(s)** | **Milestone** |
|-------------|---------------|
| 1           | Finalize system architecture & components |
| 2–3         | Set up IMU sensors on ring and wristband, test readings |
| 4           | Implement BLE communication between modules |
| 5           | Design feedback mechanism (vibration alerts on bad posture) |
| 6–7         | Collect usage data (CSV logs) for initial analysis |
| 8           | Begin AI model training (basic classifier) |
| 9           | Integrate model into backend or on-device inference |
| 10          | Develop simple mobile/web dashboard (optional) |
| 11          | System testing with real-world usage |
| 12          | Final testing, tuning, and documentation for submission/demo |

---

## 💡 Future Improvements (Optional)
- Add hand PWV (Pulse Wave Velocity) sensor for circulation analysis *(Need further research)*
- Include a grip force sensor to measure strain
- Real-time app feedback with ergonomics suggestions

---
