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

## Block diagram

<img width="1930" height="820" alt="Archi_CTS" src="https://github.com/user-attachments/assets/e1172107-2ac0-4548-b6df-67cd9aa3bca0" />


---

## 🧩 System Components (Phase I)

| Module/Component        | Details                                                                                       |
|------------------------|-----------------------------------------------------------------------------------------------|
| **Wristband Module**   | - IMU Sensor: Tracks wrist orientation & motion<br>- ESP32 MCU / ESP32-C3: Processes data and handles BLE/WiFi<br>- Vibration Motor: Haptic feedback for incorrect posture<br>- 401012 Li-Po Battery + Charger: Portable power supply |
| **Ring Module**        | - IMU Sensor (MPU6050/MPU9250): Detects finger movement<br>- 401012 Li-Po Battery: Lightweight and wearable |
| **Communication**      | - Bluetooth Low Energy (BLE) between ring and wristband<br>- Optionally sends data to external app via BLE/Wi-Fi |

---

## 🧩 Software Components (Phase II)

| Component        | Details                                                                                       |
|------------------------|-----------------------------------------------------------------------------------------|
| **Mobile Application**  | - Dashboard:<br> &nbsp;&nbsp; • Live posture monitoring<br> &nbsp;&nbsp; • Posture tracked with time<br> &nbsp;&nbsp; • Alert history & feedback |
| **Database**   | - TimeScaleDB (A Time series database) |
| **AI Model** | - Model Type: Decision Tree / SVM / LSTM (TBD)<br>- Input Features:<br> &nbsp;&nbsp; • Wrist angle + duration<br> &nbsp;&nbsp; • Finger motion frequency<br> &nbsp;&nbsp; • PPG parameters (Future Implementation)<br>- Output: Risk score or posture alert<br>- Model Training: Local/cloud-based using labeled data<br>- Deployment: On-device (threshold logic) or app (model inference) |


---


## 💡 Future Improvements
- Get PPG sensor output for further data analysis for CTS detection
- Add hand PWV (Pulse Wave Velocity) sensor for circulation analysis *(Need further research)*
- Real-time app feedback with ergonomics suggestions

---
