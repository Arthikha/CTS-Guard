# 📎 CTS-Guard

## Project Overview

Carpal Tunnel Syndrome (CTS) is a common repetitive strain injury caused by prolonged or improper wrist and finger movement, especially during tasks like typing.

This project aims to design a wearable system consisting of a **wristband and a ring like structure** to monitor hand and wrist activity patterns for early detection of CTS.  

The system uses **IMU sensors** on both the wrist and finger to detect:  

- Repetitive finger movements  
- Static or improper wrist angles  
- Prolonged usage patterns

An AI-based classifier can be used later to analyze collected data to detect CTS risk.


---

## Prototype
<img width="356" height="272" alt="CTSdiag" src="https://github.com/user-attachments/assets/7b5d0402-b6c1-4d12-a442-f633e4c05593" />


---

## Block diagram

<img width="2060" height="820" alt="CTS" src="https://github.com/user-attachments/assets/a42cae6e-4557-4e47-a1a8-5d7e658fc397" />


---

## 🧩 System Components (Phase I)

| Module/Component        | Details                                                                                       |
|------------------------|-----------------------------------------------------------------------------------------------|
| **Wristband Module**   | - IMU Sensor (BMI160): Tracks wrist orientation & motion<br>- ESP32-C3: Processes data and handles BLE<br>- Vibration Motor: Haptic feedback for incorrect posture<br>- 401012 Li-Po Battery + Charger: Portable power supply |
| **Ring Module**        | - IMU Sensor (BMI160): Detects finger movement |
| **Communication**      | - Bluetooth Low Energy (BLE) between wristband and mobile |

---

## 🧩 Software Components (Phase II)

| Component        | Details                                                                                       |
|------------------------|-----------------------------------------------------------------------------------------|
| **Mobile Application**  | - Dashboard:<br> &nbsp;&nbsp; • Live posture monitoring<br> &nbsp;&nbsp; • Posture tracked with time<br> &nbsp;&nbsp; • Alert history & feedback |
| **Database**   | - TimeScaleDB (A Time series database) |

---


## 💡 Future Improvements
- Get PPG sensor output for further data analysis for CTS detection
- Add hand PWV (Pulse Wave Velocity) sensor for circulation analysis *(Need further research)*
- AI Model - Could use LSTM and GAN for predicting risk scores and posture alerts


---
