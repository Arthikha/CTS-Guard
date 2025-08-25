# 📎 CTS-prevention-device

## Project Overview

Carpal Tunnel Syndrome (CTS) is a common repetitive strain injury caused by prolonged or improper wrist and finger movement, especially during tasks like typing.

This project aims to design a wearable system consisting of a **wristband and a ring** to monitor hand and wrist activity patterns for early detection of CTS.  

The system uses **IMU sensors** on both the wrist and finger to detect:  

- Repetitive finger movements  
- Static or improper wrist angles  
- Prolonged usage patterns  

Additionally, the system may incorporate **hand PPG sensors (e.g., MAX30102)** to monitor circulation and complement movement data. An AI-based classifier will later analyze collected data to detect CTS risk.

---

## 🧩 System Components & AI Scope

| Module/Component        | Details                                                                                       |
|------------------------|-----------------------------------------------------------------------------------------------|
| **Wristband Module**   | - IMU Sensor: Tracks wrist orientation & motion<br>- ESP32 MCU / ESP32-C3: Processes data and handles BLE/WiFi<br>- Vibration Motor: Haptic feedback for incorrect posture<br>- 401012 Li-Po Battery + Charger: Portable power supply |
| **Ring Module**        | - IMU Sensor (MPU6050/MPU9250): Detects finger movement<br>- Low-power MCU (ESP32-C3/nRF52832): BLE communication<br>- 401012 Li-Po Battery: Lightweight and wearable |
| **Communication**      | - Bluetooth Low Energy (BLE) between ring and wristband<br>- Optionally sends data to external app via BLE/Wi-Fi |
| **AI & Application**   | - Data collected & labeled for AI model training (risky usage classification)<br>- Optional mobile/web dashboard:<br> &nbsp;&nbsp; • Live posture monitoring<br> &nbsp;&nbsp; • Movement heatmaps<br> &nbsp;&nbsp; • Alert history & feedback |
| **AI Model Scope (Phase II)** | - Model Type: Decision Tree / SVM / LSTM (TBD)<br>- Input Features:<br> &nbsp;&nbsp; • Wrist angle + duration<br> &nbsp;&nbsp; • Finger motion frequency<br> &nbsp;&nbsp; • PPG parameters<br>- Output: Risk score or posture alert<br>- Model Training: Local/cloud-based using labeled data<br>- Deployment: On-device (threshold logic) or app (model inference) |

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
| 7–8  | Collect preliminary usage data (CSV logs) from tests | ⏳ Planned |
| 8-9  | Begin AI model training using collected data | ⏳ Not started |
| 9-10 | Integrate AI model into backend or on-device inference | ⏳ Not started |
| 10–11 | Develop simple mobile/web dashboard for visualization and alerts | ⏳ Not started |
| 11–12 | Integrate PPG/MAX30102 sensor and verify reading |⏳ Planned |
| 12–13 | System testing, tuning, and documentation for submission/demo | ⏳ Not started |

---

## 💡 Future Improvements (Optional)
- Add hand PWV (Pulse Wave Velocity) sensor for circulation analysis *(Need further research)*
- Real-time app feedback with ergonomics suggestions

---
