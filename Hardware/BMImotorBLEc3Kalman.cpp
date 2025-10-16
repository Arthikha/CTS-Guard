#include <Arduino.h>
#include <Wire.h>
#include <BMI160Gen.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// -------------------- Hardware pins --------------------
#define MOTOR_PIN 10     // GPIO10 for motor control
#define I2C_SDA   4
#define I2C_SCL   5

// -------------------- BMI160 addresses -----------------
#define BMI_ADDR_WRIST  0x68
#define BMI_ADDR_FINGER 0x69

BMI160GenClass bmi_wrist;
BMI160GenClass bmi_finger;

// -------------------- BLE ------------------------------
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
BLECharacteristic *pCharacteristic;

// -------------------- Units & scales --------------------
// NOTE: these match your Python
static const float ACC_LSB_PER_G   = 16384.0f;   // ±2g
static const float GYRO_LSB_PER_DPS= 16.4f;      // ±2000 dps
static const float G0               = 9.80665f;  // m/s^2
static const float DEG2RAD          = 3.14159265358979323846f / 180.0f;

// -------------------- Stationary logic -----------------
#define STATIONARY_TIME_MS 5000UL
#define STOP_DELAY_MS      1000UL

unsigned long motorStartTime = 0;
bool motorOn = false;

unsigned long wristStillStart = 0;
bool wristStationary = false;

// Use **filtered SI** thresholds now
// accel magnitude ~ g (± tolerance), low gyro
static const float ACC_MAG_MIN = 9.3f;   // m/s²
static const float ACC_MAG_MAX = 10.3f;  // m/s²
static const float GYRO_THR    = 0.2f;   // rad/s per-axis

// -------------------- Kalman filter --------------------
struct Kalman1D {
  float x;   // state estimate
  float P;   // estimate covariance
  float Q;   // process noise
  float R;   // measurement noise
  bool  init;

  void initWith(float x0, float P0, float Q_, float R_) {
    x = x0; P = P0; Q = Q_; R = R_; init = true;
  }
  float update(float z) {
    // Predict
    float x_pred = x;
    float P_pred = P + Q;
    // Update
    float K = P_pred / (P_pred + R);
    x = x_pred + K * (z - x_pred);
    P = (1.0f - K) * P_pred;
    return x;
  }
};

// 6 axes per sensor for acc+gyro
struct IMUFilters {
  Kalman1D acc[3];
  Kalman1D gyr[3];
};

// -------------------- Bias storage ---------------------
struct Biases {
  float acc[3];   // m/s²
  float gyr[3];   // rad/s
};

// One set for wrist + one for finger
IMUFilters kf_wrist, kf_finger;
Biases     bias_wrist = {{0,0,0},{0,0,0}};
Biases     bias_finger= {{0,0,0},{0,0,0}};
bool       calibrated_wrist  = false;
bool       calibrated_finger = false;

// -------------------- Helpers --------------------------
static inline void rawToSI(int ax, int ay, int az, int gx, int gy, int gz,
                           float &ax_m, float &ay_m, float &az_m,
                           float &gx_r, float &gy_r, float &gz_r) {
  ax_m = (ax / ACC_LSB_PER_G) * G0;
  ay_m = (ay / ACC_LSB_PER_G) * G0;
  az_m = (az / ACC_LSB_PER_G) * G0;

  float gx_dps = gx / GYRO_LSB_PER_DPS;
  float gy_dps = gy / GYRO_LSB_PER_DPS;
  float gz_dps = gz / GYRO_LSB_PER_DPS;
  gx_r = gx_dps * DEG2RAD;
  gy_r = gy_dps * DEG2RAD;
  gz_r = gz_dps * DEG2RAD;
}

static inline float vmag(float x, float y, float z) {
  return sqrtf(x*x + y*y + z*z);
}

static bool isStationarySI(float ax, float ay, float az, float gx, float gy, float gz) {
  float aMag = vmag(ax, ay, az);
  bool gyroQuiet = (fabsf(gx) < GYRO_THR) && (fabsf(gy) < GYRO_THR) && (fabsf(gz) < GYRO_THR);
  bool accelNearG = (aMag > ACC_MAG_MIN) && (aMag < ACC_MAG_MAX);
  return gyroQuiet && accelNearG;
}

// Simple startup calibration: average N stationary samples then
// set acc bias so that mean equals expected gravity [0,0,g] (device vertical-ish).
// Gyro bias = mean.
void computeBiases(BMI160GenClass &imu, Biases &B, const char *tag) {
  const uint16_t CAL_SAMPLES = 400;  // ~400*5ms = 2s
  uint16_t n = 0;

  double sax=0, say=0, saz=0, sgx=0, sgy=0, sgz=0;
  uint16_t stationary_hits = 0;

  while (n < CAL_SAMPLES) {
    int ax, ay, az, gx, gy, gz;
    imu.readAccelerometer(ax, ay, az);
    imu.readGyro(gx, gy, gz);

    float ax_m, ay_m, az_m, gx_r, gy_r, gz_r;
    rawToSI(ax, ay, az, gx, gy, gz, ax_m, ay_m, az_m, gx_r, gy_r, gz_r);

    // accumulate
    sax += ax_m; say += ay_m; saz += az_m;
    sgx += gx_r; sgy += gy_r; sgz += gz_r;

    if (isStationarySI(ax_m, ay_m, az_m, gx_r, gy_r, gz_r)) stationary_hits++;

    delay(5);
    n++;
  }

  // Means
  float axb = sax / CAL_SAMPLES;
  float ayb = say / CAL_SAMPLES;
  float azb = saz / CAL_SAMPLES;
  float gxb = sgx / CAL_SAMPLES;
  float gyb = sgy / CAL_SAMPLES;
  float gzb = sgz / CAL_SAMPLES;

  // Target gravity along +Z (like your Python)
  B.acc[0] = axb - 0.0f;
  B.acc[1] = ayb - 0.0f;
  B.acc[2] = azb - G0;

  B.gyr[0] = gxb;
  B.gyr[1] = gyb;
  B.gyr[2] = gzb;

  Serial.printf("[%s] Calibration done. Stationary hits: %u/%u\n", tag, stationary_hits, CAL_SAMPLES);
  Serial.printf("[%s] Acc bias (m/s^2): %.4f, %.4f, %.4f\n", tag, B.acc[0], B.acc[1], B.acc[2]);
  Serial.printf("[%s] Gyr bias (rad/s): %.5f, %.5f, %.5f\n", tag, B.gyr[0], B.gyr[1], B.gyr[2]);
}

// Initialize per-axis Kalman filters with your Python Q/R
void initKalman(IMUFilters &F, const float acc_init[3], const float gyr_init[3]) {
  // Accelerometer
  for (int i=0;i<3;i++) {
    F.acc[i].initWith(acc_init[i], 1.0f, 1e-5f, 0.1f);
  }
  // Gyro
  for (int i=0;i<3;i++) {
    F.gyr[i].initWith(gyr_init[i], 1.0f, 1e-6f, 0.01f);
  }
}

// Pass SI units, subtract biases, then filter
void filterAndBiasCorrect(const float aSI[3], const float gSI[3],
                          const Biases &B, IMUFilters &F,
                          float aOut[3], float gOut[3]) {
  // bias removal
  float ax = aSI[0] - B.acc[0];
  float ay = aSI[1] - B.acc[1];
  float az = aSI[2] - B.acc[2];

  float gx = gSI[0] - B.gyr[0];
  float gy = gSI[1] - B.gyr[1];
  float gz = gSI[2] - B.gyr[2];

  // Kalman
  aOut[0] = F.acc[0].update(ax);
  aOut[1] = F.acc[1].update(ay);
  aOut[2] = F.acc[2].update(az);

  gOut[0] = F.gyr[0].update(gx);
  gOut[1] = F.gyr[1].update(gy);
  gOut[2] = F.gyr[2].update(gz);
}

// -------------------- Setup ----------------------------
void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("Initializing...");

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  Wire.begin(I2C_SDA, I2C_SCL, 100000);

  if (!bmi_wrist.begin(BMI160GenClass::I2C_MODE, BMI_ADDR_WRIST)) {
    Serial.println("Failed to find Wrist BMI160");
  } else {
    Serial.println("Wrist BMI160 Found!");
  }

  if (!bmi_finger.begin(BMI160GenClass::I2C_MODE, BMI_ADDR_FINGER)) {
    Serial.println("Failed to find Finger BMI160");
  } else {
    Serial.println("Finger BMI160 Found!");
  }

  // -------- BLE --------
  BLEDevice::init("ESP32_IMU_BMI160");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE Advertising started.");

  // -------- Calibration --------
  Serial.println("Hold both sensors still for ~2 seconds each...");
  computeBiases(bmi_wrist,  bias_wrist,  "WRIST");
  computeBiases(bmi_finger, bias_finger, "FINGER");
  calibrated_wrist  = true;
  calibrated_finger = true;

  // Initialize Kalman with initial estimates = (bias-corrected) current readings
  auto seedInit = [&](BMI160GenClass &imu, const Biases &B, IMUFilters &F, const char* tag){
    int ax, ay, az, gx, gy, gz;
    imu.readAccelerometer(ax, ay, az);
    imu.readGyro(gx, gy, gz);
    float ax_m, ay_m, az_m, gx_r, gy_r, gz_r;
    rawToSI(ax, ay, az, gx, gy, gz, ax_m, ay_m, az_m, gx_r, gy_r, gz_r);
    float a0[3] = { ax_m - B.acc[0], ay_m - B.acc[1], az_m - B.acc[2] };
    float g0[3] = { gx_r - B.gyr[0], gy_r - B.gyr[1], gz_r - B.gyr[2] };
    initKalman(F, a0, g0);
    Serial.printf("[%s] Kalman initialized.\n", tag);
  };
  seedInit(bmi_wrist,  bias_wrist,  kf_wrist,  "WRIST");
  seedInit(bmi_finger, bias_finger, kf_finger, "FINGER");

  Serial.println("Setup complete.\n");
}

// -------------------- Loop -----------------------------
void loop() {
  delay(50); // faster loop helps filtering smoothness

  // Read raw ints
  int axW_i, ayW_i, azW_i, gxW_i, gyW_i, gzW_i;
  int axF_i, ayF_i, azF_i, gxF_i, gyF_i, gzF_i;

  bmi_wrist.readAccelerometer(axW_i, ayW_i, azW_i);
  bmi_wrist.readGyro(gxW_i, gyW_i, gzW_i);

  bmi_finger.readAccelerometer(axF_i, ayF_i, azF_i);
  bmi_finger.readGyro(gxF_i, gyF_i, gzF_i);

  // Convert to SI
  float awSI[3], gwSI[3], afSI[3], gfSI[3];
  rawToSI(axW_i, ayW_i, azW_i, gxW_i, gyW_i, gzW_i,
          awSI[0], awSI[1], awSI[2], gwSI[0], gwSI[1], gwSI[2]);
  rawToSI(axF_i, ayF_i, azF_i, gxF_i, gyF_i, gzF_i,
          afSI[0], afSI[1], afSI[2], gfSI[0], gfSI[1], gfSI[2]);

  // Bias-correct + Kalman filter
  float awKF[3], gwKF[3], afKF[3], gfKF[3];
  filterAndBiasCorrect(awSI, gwSI, bias_wrist,  kf_wrist,  awKF, gwKF);
  filterAndBiasCorrect(afSI, gfSI, bias_finger, kf_finger, afKF, gfKF);

  // ------------- Stationary / motor logic uses FILTERED SI -------------
  static uint16_t movingCount = 0, stillCount = 0;
  unsigned long now = millis();

  if (isStationarySI(awKF[0], awKF[1], awKF[2], gwKF[0], gwKF[1], gwKF[2])) {
    stillCount++;
    movingCount = 0;
    if (!wristStationary && (stillCount * 50 >= STATIONARY_TIME_MS)) {
      wristStationary = true;
      Serial.println("Wrist is stationary (confirmed).");
    }
  } else {
    movingCount++;
    stillCount = 0;
    if (wristStationary && movingCount >= 3) {
      wristStationary = false;
      if (motorOn) {
        motorStartTime = now;
        Serial.println("Wrist moved: scheduling motor stop.");
      }
    }
  }

  // Finger movement check (filtered)
  bool fingerMoving = !isStationarySI(afKF[0], afKF[1], afKF[2], gfKF[0], gfKF[1], gfKF[2]);
  if (wristStationary && fingerMoving) {
    if (!motorOn) {
      digitalWrite(MOTOR_PIN, HIGH);
      motorOn = true;
      Serial.println("Motor ON (finger moved, wrist still).");
    }
  }

  if (motorOn && !wristStationary && (now - motorStartTime >= STOP_DELAY_MS)) {
    digitalWrite(MOTOR_PIN, LOW);
    motorOn = false;
    Serial.println("Motor OFF.");
  }

  // ------------- Output (filtered, bias-corrected SI units) -------------
  char buf[256];
  // Trim precision to keep BLE packet small
  snprintf(buf, sizeof(buf),
          //  "Wrist A[%.3f,%.3f,%.3f]m/s2 G[%.3f,%.3f,%.3f]rad/s | "
          //  "Finger A[%.3f,%.3f,%.3f]m/s2 G[%.3f,%.3f,%.3f]rad/s",
          //  awKF[0], awKF[1], awKF[2], gwKF[0], gwKF[1], gwKF[2],
          //  afKF[0], afKF[1], afKF[2], gfKF[0], gfKF[1], gfKF[2]);
          "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
           awKF[0], awKF[1], awKF[2], gwKF[0], gwKF[1], gwKF[2],
           afKF[0], afKF[1], afKF[2], gfKF[0], gfKF[1], gfKF[2]);

  Serial.println(buf);
  pCharacteristic->setValue((uint8_t*)buf, strlen(buf));
  pCharacteristic->notify();
}
