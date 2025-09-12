#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// === Motor Pin ===
#define MOTOR_PIN 4

// === I2C pins (ESP32 default) ===
#define I2C_SDA 21
#define I2C_SCL 22

// === MPU6050 Objects ===
Adafruit_MPU6050 mpuWrist;
Adafruit_MPU6050 mpuFinger;

// === Kalman Filter Class ===
class KalmanFilter {
  public:
    KalmanFilter(float q, float r, float p = 1, float x = 0) {
      Q = q; R = r; P = p; X = x;
    }
    float update(float measurement) {
      P = P + Q;
      K = P / (P + R);
      X = X + K * (measurement - X);
      P = (1 - K) * P;
      return X;
    }
  private:
    float Q, R, P, K, X;
};

// Wrist filters
KalmanFilter kfAccX_w(1e-5, 0.1), kfAccY_w(1e-5, 0.1), kfAccZ_w(1e-5, 0.1);
KalmanFilter kfGyroX_w(1e-5, 0.1), kfGyroY_w(1e-5, 0.1), kfGyroZ_w(1e-5, 0.1);

// Finger filters
KalmanFilter kfAccX_f(1e-5, 0.1), kfAccY_f(1e-5, 0.1), kfAccZ_f(1e-5, 0.1);
KalmanFilter kfGyroX_f(1e-5, 0.1), kfGyroY_f(1e-5, 0.1), kfGyroZ_f(1e-5, 0.1);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
      delay(3000); // wait for serial port to connect. Needed for native USB
  }
  delay(5000);

  Serial.println("Initializing...");


  // Start I2C slower (100kHz)
  Wire.begin(I2C_SDA, I2C_SCL, 100000);

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  // Initialize Wrist MPU (0x68)
  if (!mpuWrist.begin(0x68)) {
    Serial.println("Failed to find Wrist MPU6050!");
  } else {
    Serial.println("Wrist MPU6050 Found!");
    mpuWrist.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpuWrist.setGyroRange(MPU6050_RANGE_500_DEG);
  }

  // Initialize Finger MPU (0x69)
  if (!mpuFinger.begin(0x69)) {
    Serial.println("Failed to find Finger MPU6050!");
  } else {
    Serial.println("Finger MPU6050 Found!");
    mpuFinger.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpuFinger.setGyroRange(MPU6050_RANGE_500_DEG);
  }

}

// vibration trigger
void vibrateMotor(int ms) {
  digitalWrite(MOTOR_PIN, HIGH);
  delay(ms);
  digitalWrite(MOTOR_PIN, LOW);
}

void loop() {
  sensors_event_t a_w, g_w, temp_w;
  sensors_event_t a_f, g_f, temp_f;

  // === Read Wrist ===
  if (mpuWrist.getEvent(&a_w, &g_w, &temp_w)) {
    float ax_w = kfAccX_w.update(a_w.acceleration.x);
    float ay_w = kfAccY_w.update(a_w.acceleration.y);
    float az_w = kfAccZ_w.update(a_w.acceleration.z);
    float gx_w = kfGyroX_w.update(g_w.gyro.x);
    float gy_w = kfGyroY_w.update(g_w.gyro.y);
    float gz_w = kfGyroZ_w.update(g_w.gyro.z);

    Serial.printf("Wrist Accel: %.2f, %.2f, %.2f | Gyro: %.2f, %.2f, %.2f\n",
                  ax_w, ay_w, az_w, gx_w, gy_w, gz_w);

    // Threshold check for wrist
    if (fabs(gx_w) > 1.0 || fabs(gy_w) > 1.0 || fabs(gz_w) > 1.0) {
      vibrateMotor(200);
    }
  } else {
    Serial.println("Wrist read failed (I2C error).");
  }

  // === Read Finger ===
  if (mpuFinger.getEvent(&a_f, &g_f, &temp_f)) {
    float ax_f = kfAccX_f.update(a_f.acceleration.x);
    float ay_f = kfAccY_f.update(a_f.acceleration.y);
    float az_f = kfAccZ_f.update(a_f.acceleration.z);
    float gx_f = kfGyroX_f.update(g_f.gyro.x);
    float gy_f = kfGyroY_f.update(g_f.gyro.y);
    float gz_f = kfGyroZ_f.update(g_f.gyro.z);

    Serial.printf("Finger Accel: %.2f, %.2f, %.2f | Gyro: %.2f, %.2f, %.2f\n",
                  ax_f, ay_f, az_f, gx_f, gy_f, gz_f);

    // Threshold check for finger
    if (fabs(gx_f) > 1.0 || fabs(gy_f) > 1.0 || fabs(gz_f) > 1.0) {
      vibrateMotor(200);
      delay(5000);
    }
  } else {
    Serial.println("Finger read failed (I2C error).");
  }

  delay(50); // small delay for bus stability
}
