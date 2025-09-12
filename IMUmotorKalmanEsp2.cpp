#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// === Motor Pin ===
#define MOTOR_PIN 4   // GPIO4 for motor control

// === I2C pins for ESP32 ===
#define I2C_SDA 21
#define I2C_SCL 22

// === MPU objects ===
Adafruit_MPU6050 mpu_wrist;
Adafruit_MPU6050 mpu_finger;

// === MPU addresses ===
#define MPU_ADDR_WRIST 0x69  // AD0 -> 3.3V
#define MPU_ADDR_FINGER 0x68 // AD0 -> GND

// === Movement detection thresholds ===
#define ACC_THRESHOLD 0.8       // m/s^2
#define GYRO_THRESHOLD 0.1      // rad/s
#define STATIONARY_TIME 3000UL  // 3 sec
#define STOP_DELAY 1000         // 1 sec after wrist moves

unsigned long wristStillStart = 0;
bool wristStationary = false;
bool motorOn = false;
unsigned long motorStartTime = 0;

// === Simple 1D Kalman filter class ===
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

// === Kalman filters for wrist ===
KalmanFilter kfAccX_w(1e-5, 0.1), kfAccY_w(1e-5, 0.1), kfAccZ_w(1e-5, 0.1);
KalmanFilter kfGyroX_w(1e-6, 0.01), kfGyroY_w(1e-6, 0.01), kfGyroZ_w(1e-6, 0.01);

// === Kalman filters for finger ===
KalmanFilter kfAccX_f(1e-5, 0.1), kfAccY_f(1e-5, 0.1), kfAccZ_f(1e-5, 0.1);
KalmanFilter kfGyroX_f(1e-6, 0.01), kfGyroY_f(1e-6, 0.01), kfGyroZ_f(1e-6, 0.01);

void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.println("Initializing...");

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  Wire.begin(I2C_SDA, I2C_SCL, 100000);  // start I2C at 100kHz

  // === Finger MPU ===
  if (!mpu_finger.begin(MPU_ADDR_FINGER, &Wire)) {
    Serial.println("Failed to find Finger MPU6050");
  } else {
    Serial.println("Finger MPU6050 Found!");
    mpu_finger.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu_finger.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu_finger.setFilterBandwidth(MPU6050_BAND_5_HZ);
  }

  // === Wrist MPU ===
  if (!mpu_wrist.begin(MPU_ADDR_WRIST, &Wire)) {
    Serial.println("Failed to find Wrist MPU6050");
  } else {
    Serial.println("Wrist MPU6050 Found!");
    mpu_wrist.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu_wrist.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu_wrist.setFilterBandwidth(MPU6050_BAND_5_HZ);
  }

  Serial.println("Setup complete.\n");
}

// === Stationary check with filtered data ===
bool isStationary(float ax, float ay, float az, float gx, float gy, float gz) {
  Serial.println(fabs(ax));
  return (fabs(ax) < ACC_THRESHOLD &&
          fabs(ay) < ACC_THRESHOLD &&
          fabs(az - 9.8) < 0.5 &&   // close to gravity
          fabs(gx) < GYRO_THRESHOLD &&
          fabs(gy) < GYRO_THRESHOLD &&
          fabs(gz) < GYRO_THRESHOLD);
}

bool isMoving(float ax, float ay, float az, float gx, float gy, float gz) {
  return !isStationary(ax, ay, az, gx, gy, gz);
}

void loop() {
  sensors_event_t aW, gW, tW;
  sensors_event_t aF, gF, tF;

  mpu_wrist.getEvent(&aW, &gW, &tW);
  delay(5);
  mpu_finger.getEvent(&aF, &gF, &tF);

  unsigned long now = millis();

  // === Apply Kalman filter (wrist) ===
  float ax_w = kfAccX_w.update(aW.acceleration.x);
  float ay_w = kfAccY_w.update(aW.acceleration.y);
  float az_w = kfAccZ_w.update(aW.acceleration.z);
  float gx_w = kfGyroX_w.update(gW.gyro.x);
  float gy_w = kfGyroY_w.update(gW.gyro.y);
  float gz_w = kfGyroZ_w.update(gW.gyro.z);

  // === Apply Kalman filter (finger) ===
  float ax_f = kfAccX_f.update(aF.acceleration.x);
  float ay_f = kfAccY_f.update(aF.acceleration.y);
  float az_f = kfAccZ_f.update(aF.acceleration.z);
  float gx_f = kfGyroX_f.update(gF.gyro.x);
  float gy_f = kfGyroY_f.update(gF.gyro.y);
  float gz_f = kfGyroZ_f.update(gF.gyro.z);

  // === Print filtered IMU data ===
  Serial.printf("Wrist Accel: %.2f, %.2f, %.2f | Gyro: %.2f, %.2f, %.2f\n",
                ax_w, ay_w, az_w, gx_w, gy_w, gz_w);

  Serial.printf("Finger Accel: %.2f, %.2f, %.2f | Gyro: %.2f, %.2f, %.2f\n",
                ax_f, ay_f, az_f, gx_f, gy_f, gz_f);

  // === Wrist stationary tracking ===
  if (isStationary(ax_w, ay_w, az_w, gx_w, gy_w, gz_w)) {
    if (!wristStationary) {
      if (wristStillStart == 0) wristStillStart = now; // start timing
      if (now - wristStillStart >= STATIONARY_TIME) {
        wristStationary = true;
        Serial.println("Wrist is stationary (confirmed).");
      }
    }
  } else {
    wristStillStart = 0;
    if (wristStationary) {
      wristStationary = false;
      if (motorOn) {
        motorStartTime = now; // schedule stop
        Serial.println("Wrist moved: scheduling motor stop.");
      }
    }
  }

  // === Finger movement check ===
  if (wristStationary && isMoving(ax_f, ay_f, az_f, gx_f, gy_f, gz_f)) {
    if (!motorOn) {
      digitalWrite(MOTOR_PIN, HIGH);
      motorOn = true;
      Serial.println("Motor ON (finger moved, wrist still).");
    }
  }

  // === Stop motor after wrist movement + delay ===
  if (motorOn && !wristStationary && (now - motorStartTime >= STOP_DELAY)) {
    digitalWrite(MOTOR_PIN, LOW);
    motorOn = false;
    Serial.println("Motor OFF.");
  }

  delay(200); // sampling interval
}
