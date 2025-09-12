#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define MOTOR_PIN 4   // GPIO4 for motor control

// I2C pins for ESP32
#define I2C_SDA 21
#define I2C_SCL 22

Adafruit_MPU6050 mpu_wrist;
Adafruit_MPU6050 mpu_finger;

// MPU addresses
#define MPU_ADDR_WRIST 0x69  // AD0 -> 3.3V
#define MPU_ADDR_FINGER 0x68 // AD0 -> GND

// Movement detection thresholds
#define ACC_THRESHOLD 0.8       // m/s^2
#define GYRO_THRESHOLD 0.1     // rad/s
#define STATIONARY_TIME 3000UL  // 5 sec for testing
#define STOP_DELAY 1000         // 1 sec after wrist moves

unsigned long wristStillStart = 0;
bool wristStationary = false;
bool motorOn = false;
unsigned long motorStartTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
      delay(3000); // wait for serial port to connect. Needed for native USB
  }
  delay(5000);

  Serial.println("Initializing...");

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  Wire.begin(I2C_SDA, I2C_SCL, 100000);  // SDA=21, SCL=22, 100kHz

  // Finger IMU
  if (!mpu_finger.begin(MPU_ADDR_FINGER, &Wire)) {
    Serial.println("Failed to find Finger MPU6050");
    // while (1) delay(10);
  }else{
    Serial.println("Finger MPU6050 Found!");
    mpu_finger.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu_finger.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu_finger.setFilterBandwidth(MPU6050_BAND_5_HZ);
  }
  

  // Wrist IMU
  if (!mpu_wrist.begin(MPU_ADDR_WRIST, &Wire)) {
    Serial.println("Failed to find Wrist MPU6050");
    // while (1) delay(10);
  }else{
    Serial.println("Wrist MPU6050 Found!");
    mpu_wrist.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu_wrist.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu_wrist.setFilterBandwidth(MPU6050_BAND_5_HZ);
  }

  Serial.println("Setup complete.\n");
}

bool isStationary(sensors_event_t &a, sensors_event_t &g) {
  Serial.println("isStationary called");
  return (fabs(a.acceleration.x) < ACC_THRESHOLD &&
          fabs(a.acceleration.y) < ACC_THRESHOLD &&
          fabs(a.acceleration.z - 9.8) < 0.5 && // near gravity
          fabs(g.gyro.x) < GYRO_THRESHOLD &&
          fabs(g.gyro.y) < GYRO_THRESHOLD &&
          fabs(g.gyro.z) < GYRO_THRESHOLD);
}

bool isMoving(sensors_event_t &a, sensors_event_t &g) {
  Serial.println("isMoving called");
  return !isStationary(a, g);
}

void loop() {
  sensors_event_t aW, gW, tW;
  sensors_event_t aF, gF, tF;

  mpu_wrist.getEvent(&aW, &gW, &tW);
  delay(5);
  mpu_finger.getEvent(&aF, &gF, &tF);

  unsigned long now = millis();

   // ---- Print wrist IMU data ----
  Serial.print("Wrist Accel: ");
  Serial.print(aW.acceleration.x); Serial.print(", ");
  Serial.print(aW.acceleration.y); Serial.print(", ");
  Serial.print(aW.acceleration.z);
  Serial.print(" | Gyro: ");
  Serial.print(gW.gyro.x); Serial.print(", ");
  Serial.print(gW.gyro.y); Serial.print(", ");
  Serial.println(gW.gyro.z);

  // ---- Print finger IMU data ----
  Serial.print("Finger Accel: ");
  Serial.print(aF.acceleration.x); Serial.print(", ");
  Serial.print(aF.acceleration.y); Serial.print(", ");
  Serial.print(aF.acceleration.z);
  Serial.print(" | Gyro: ");
  Serial.print(gF.gyro.x); Serial.print(", ");
  Serial.print(gF.gyro.y); Serial.print(", ");
  Serial.println(gF.gyro.z);
  
  

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

  // Finger movement check
  if (wristStationary && isMoving(aF, gF)) {
    if (!motorOn) {
      digitalWrite(MOTOR_PIN, HIGH);
      motorOn = true;
      Serial.println("Motor ON (finger moved, wrist still).");
    }
  }

  // Stop motor after wrist movement + delay
  if (motorOn && !wristStationary && (now - motorStartTime >= STOP_DELAY)) {
    digitalWrite(MOTOR_PIN, LOW);
    motorOn = false;
    Serial.println("Motor OFF.");
  }

  delay(200); // sampling interval
}
