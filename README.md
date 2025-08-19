# 📎 CTS-prevention-device

## Project Overview

Carpal Tunnel Syndrome (CTS) is a common repetitive strain injury caused by prolonged or improper wrist and finger movement, especially during tasks like typing or using digital devices.  

This project aims to design a wearable system consisting of a **wristband and a ring** to monitor hand and wrist activity patterns for early detection and prevention of CTS.  

The system uses **IMU sensors** on both the wrist and finger to detect:  

- Repetitive finger movements  
- Static or improper wrist angles  
- Prolonged usage patterns  

Additionally, the system may incorporate **hand PPG sensors (e.g., MAX30102)** to monitor circulation and complement movement data. An AI-based classifier will later analyze collected data to detect CTS risk levels and notify the user. A mobile or web application will provide visualization and feedback.

---


## 🧩 System Components

### 1. Wristband Module
- **IMU Sensor**: Tracks wrist orientation & motion
- **ESP32 MCU/ ESP32-C3**: Processes data and handles BLE and WiFi communication
- **Vibration Motor**: Provides haptic feedback if posture is incorrect
- **401012 Li-Po Battery + Charger**: Portable power supply

### 2. Ring Module
- **IMU Sensor** (e.g., MPU6050 or MPU9250): Detects finger movement
- **Low-power MCU** (ESP32-C3 or nRF52832): Bluetooth communication
- **401012 LiPo Battery**: Lightweight and wearable

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
  - PPG parameters
- Output: Risk score or posture alert
- Model Training: Local or cloud-based using collected labeled data
- Deployment: On-device (basic threshold logic) or app (model inference)

---

## Block diagram

<img width="1880" height="760" alt="Ana" src="https://github.com/user-attachments/assets/2ce5b96a-18f3-45b1-af5c-67bcaa807c89" />






## 📅 Project Timeline (3-Month Plan)

| Week | Milestone | Status |
|------|-----------|--------|
| 1    | Finalize system architecture & components | ✅ Completed |
| 2–3  | Set up IMU sensors on wristband and ring, test readings | ✅ Completed |
| 4    | Implement BLE communication between modules | ✅ Completed |
| 5–6  | Test and calibrate IMU sensors on wristband and ring | 🔄 In progress |
| 6–7  | Test and integrate vibration motor for feedback | ❌ Not working yet |
| 7–8  | Integrate PPG/MAX30102 sensor and verify readings | ⚙️ Planned |
| 8–9  | Collect preliminary usage data (CSV logs) from tests | ⏳ Planned |
| 9–10 | Begin AI model training using collected data | ⏳ Not started |
| 10–11 | Integrate AI model into backend or on-device inference | ⏳ Not started |
| 11–12 | Develop simple mobile/web dashboard for visualization and alerts | ⏳ Not started |
| 12–13 | System testing, tuning, and documentation for submission/demo | ⏳ Not started |

---

## 💡 Future Improvements (Optional)
- Add hand PWV (Pulse Wave Velocity) sensor for circulation analysis *(Need further research)*
- Real-time app feedback with ergonomics suggestions

---
