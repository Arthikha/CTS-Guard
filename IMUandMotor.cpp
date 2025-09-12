#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define MOTOR_PIN 4

Adafruit_MPU6050 mpu_wrist;
Adafruit_MPU6050 mpu_finger;

// MPU addresses
#define MPU_ADDR_WRIST 0x69  // AD0 -> 3.3V
#define MPU_ADDR_FINGER 0x68 // AD0 -> GND

// Movement detection thresholds
#define ACC_THRESHOLD 0.2     // m/s^2 (for stillness detection)
#define GYRO_THRESHOLD 0.05   // rad/s
#define STATIONARY_TIME 1000UL   // 3600000UL  // 1 hour (ms) -> adjust for testing
#define STOP_DELAY 1000       // 1 sec after wrist moves

unsigned long wristStillStart = 0;
bool wristStationary = false;
bool motorOn = false;
unsigned long motorStartTime = 0;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Initializing...");

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  Wire.begin();

  // Wrist IMU
  if (!mpu_wrist.begin(MPU_ADDR_WRIST, &Wire)) {
    Serial.println("Failed to find Wrist MPU6050");
    while (1) delay(10);
  }
  Serial.println("Wrist MPU6050 Found!");

  // Finger IMU
  if (!mpu_finger.begin(MPU_ADDR_FINGER, &Wire)) {
    Serial.println("Failed to find Finger MPU6050");
    while (1) delay(10);
  }
  Serial.println("Finger MPU6050 Found!");

  // Common setup
  mpu_wrist.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu_wrist.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu_wrist.setFilterBandwidth(MPU6050_BAND_5_HZ);

  mpu_finger.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu_finger.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu_finger.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("Setup complete.\n");
}

bool isStationary(sensors_event_t &a, sensors_event_t &g) {
  return (fabs(a.acceleration.x) < ACC_THRESHOLD &&
          fabs(a.acceleration.y) < ACC_THRESHOLD &&
          fabs(a.acceleration.z - 9.8) < 0.5 && // near gravity
          fabs(g.gyro.x) < GYRO_THRESHOLD &&
          fabs(g.gyro.y) < GYRO_THRESHOLD &&
          fabs(g.gyro.z) < GYRO_THRESHOLD);
}

bool isMoving(sensors_event_t &a, sensors_event_t &g) {
  return !(fabs(a.acceleration.x) < ACC_THRESHOLD &&
           fabs(a.acceleration.y) < ACC_THRESHOLD &&
           fabs(a.acceleration.z - 9.8) < 0.5 &&
           fabs(g.gyro.x) < GYRO_THRESHOLD &&
           fabs(g.gyro.y) < GYRO_THRESHOLD &&
           fabs(g.gyro.z) < GYRO_THRESHOLD);
}

void loop() {
  sensors_event_t aW, gW, tW;
  sensors_event_t aF, gF, tF;

  mpu_wrist.getEvent(&aW, &gW, &tW);
  mpu_finger.getEvent(&aF, &gF, &tF);

  delay(1000);
  // Print Wrist
  Serial.print("Wrist Accel: X=");
  Serial.print(aW.acceleration.x); Serial.print(" Y=");
  Serial.print(aW.acceleration.y); Serial.print(" Z=");
  Serial.print(aW.acceleration.z);

  Serial.print(" | Gyro: X=");
  Serial.print(gW.gyro.x); Serial.print(" Y=");
  Serial.print(gW.gyro.y); Serial.print(" Z=");
  Serial.println(gW.gyro.z);

  // Print Finger
  Serial.print("Finger Accel: X=");
  Serial.print(aF.acceleration.x); Serial.print(" Y=");
  Serial.print(aF.acceleration.y); Serial.print(" Z=");
  Serial.print(aF.acceleration.z);

  Serial.print(" | Gyro: X=");
  Serial.print(gF.gyro.x); Serial.print(" Y=");
  Serial.print(gF.gyro.y); Serial.print(" Z=");
  Serial.println(gF.gyro.z);

  unsigned long now = millis();

  // Wrist stationary tracking
  if (isStationary(aW, gW)) {
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

  // Check finger movement
  if (wristStationary && isMoving(aF, gF)) {
    if (!motorOn) {
      digitalWrite(MOTOR_PIN, HIGH);
      motorOn = true;
      Serial.println("Motor ON (finger moved, wrist still).");
    }
  }

  // Stop motor after 1 sec of wrist movement
  if (motorOn && !wristStationary && (now - motorStartTime >= STOP_DELAY)) {
    digitalWrite(MOTOR_PIN, LOW);
    motorOn = false;
    Serial.println("Motor OFF.");
  }

  delay(200); // adjust as needed
}
