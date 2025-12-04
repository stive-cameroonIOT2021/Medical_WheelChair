#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>

// ================== GLOBAL CONFIG / CONSTANTS ==================

// Pins
const uint8_t CONFIG_LED_PIN          = 2;     // Onboard LED pin
const int     CONFIG_I2C_SDA_PIN      = 19;    // SDA pin
const int     CONFIG_I2C_SCL_PIN      = 23;    // SCL pin

// Startup
const uint32_t CONFIG_STARTUP_DELAY_MS = 200;  // Wait for Serial, etc.

// Calibration settings
const uint16_t CONFIG_CALIB_SAMPLE_COUNT        = 2000;  // samples for gyro calib
const uint32_t CONFIG_CALIB_SAMPLE_INTERVAL_US  = 2000;  // microseconds between samples
const uint32_t CONFIG_CALIB_BLINK_INTERVAL_MS   = 100;   // LED blink during calibration

// Filter / math constants
const float    CONFIG_RAD_TO_DEG                = 57.2957795f;
const float    CONFIG_COMPLEMENTARY_ALPHA       = 0.95f; // gyro weight
const float    CONFIG_MICROS_TO_SECONDS         = 1.0e-6f;
const float    CONFIG_INITIAL_ANGLE_DEG         = 0.0f;
const float    CONFIG_YAW_WRAP_POS_DEG          = 180.0f;
const float    CONFIG_YAW_WRAP_NEG_DEG          = -180.0f;
const float    CONFIG_FULL_TURN_DEG             = 360.0f;

// Print interval
const uint32_t CONFIG_PRINT_INTERVAL_MS         = 1000;  // print every second

// MPU status code
const uint8_t  CONFIG_MPU_STATUS_OK             = 0;     // status==0 => OK

// ================== GLOBAL STATE ==================

MPU6050 mpu(Wire);

// Calibration state machine
enum CalibState {
  CALIB_STATE_NOT_STARTED = 0,
  CALIB_STATE_RUNNING     = 1,
  CALIB_STATE_DONE        = 2
};

CalibState globalCalibState = CALIB_STATE_NOT_STARTED;

// Gyro calibration sums & offsets
uint16_t globalCalibSampleCounter = 0;
float    globalGyroXSum           = 0.0f;
float    globalGyroYSum           = 0.0f;
float    globalGyroZSum           = 0.0f;

float    globalGyroXOffset        = 0.0f;
float    globalGyroYOffset        = 0.0f;
float    globalGyroZOffset        = 0.0f;

// Timing globals
uint32_t globalLastBlinkMillis    = 0;
uint32_t globalLastSampleMicros   = 0;
uint32_t globalLastUpdateMicros   = 0;
uint32_t globalLastPrintMillis    = 0;

// Angles
float    globalPitchDeg           = 0.0f;
float    globalRollDeg            = 0.0f;
float    globalYawDeg             = 0.0f;

// IMU status flag
bool     globalImuOk              = false;

// ================== HELPER FUNCTIONS ==================

void startCalibration()
{
  globalCalibState          = CALIB_STATE_RUNNING;
  globalCalibSampleCounter  = 0;
  globalGyroXSum            = 0.0f;
  globalGyroYSum            = 0.0f;
  globalGyroZSum            = 0.0f;

  globalLastSampleMicros    = micros();
  globalLastBlinkMillis     = millis();

  digitalWrite(CONFIG_LED_PIN, LOW);

  Serial.println(F("Calibration started, DO NOT MOVE the board."));
}

void handleCalibration()
{
  if (globalCalibState != CALIB_STATE_RUNNING) {
    return;
  }

  uint32_t currentMillis = millis();
  uint32_t currentMicros = micros();

  // LED blink (non-blocking)
  if (currentMillis - globalLastBlinkMillis >= CONFIG_CALIB_BLINK_INTERVAL_MS) {
    globalLastBlinkMillis = currentMillis;
    digitalWrite(CONFIG_LED_PIN, !digitalRead(CONFIG_LED_PIN));
  }

  // Sample IMU (non-blocking)
  if ((currentMicros - globalLastSampleMicros) >= CONFIG_CALIB_SAMPLE_INTERVAL_US &&
      globalCalibSampleCounter < CONFIG_CALIB_SAMPLE_COUNT) {

    globalLastSampleMicros = currentMicros;

    mpu.update();

    float gyroX = mpu.getGyroX();
    float gyroY = mpu.getGyroY();
    float gyroZ = mpu.getGyroZ();

    globalGyroXSum += gyroX;
    globalGyroYSum += gyroY;
    globalGyroZSum += gyroZ;

    globalCalibSampleCounter++;
  }

  // Finish calibration
  if (globalCalibSampleCounter >= CONFIG_CALIB_SAMPLE_COUNT) {

    float sampleCountAsFloat = static_cast<float>(CONFIG_CALIB_SAMPLE_COUNT);

    globalGyroXOffset = globalGyroXSum / sampleCountAsFloat;
    globalGyroYOffset = globalGyroYSum / sampleCountAsFloat;
    globalGyroZOffset = globalGyroZSum / sampleCountAsFloat;

    globalCalibState = CALIB_STATE_DONE;
    digitalWrite(CONFIG_LED_PIN, LOW);

    Serial.println(F("Calibration done (gyro only)."));
    Serial.print(F("Gyro offsets: "));
    Serial.print(globalGyroXOffset);
    Serial.print(F(", "));
    Serial.print(globalGyroYOffset);
    Serial.print(F(", "));
    Serial.println(globalGyroZOffset);

    // Initialize angles from accelerometer
    mpu.update();

    float accelX = mpu.getAccX();
    float accelY = mpu.getAccY();
    float accelZ = mpu.getAccZ();

    float accelSquaredY = accelY * accelY;
    float accelSquaredZ = accelZ * accelZ;
    float accelYZNorm   = sqrtf(accelSquaredY + accelSquaredZ);

    float pitchFromAccel = atan2f(-accelX, accelYZNorm) * CONFIG_RAD_TO_DEG;
    float rollFromAccel  = atan2f( accelY, accelZ ) * CONFIG_RAD_TO_DEG;

    globalPitchDeg       = pitchFromAccel;
    globalRollDeg        = rollFromAccel;
    globalYawDeg         = CONFIG_INITIAL_ANGLE_DEG;

    globalLastUpdateMicros = micros();

    Serial.println(F("Angle reference initialized from accelerometer."));
  }
}

// ================== SETUP ==================

void setup()
{
  Serial.begin(115200);
  delay(CONFIG_STARTUP_DELAY_MS);

  pinMode(CONFIG_LED_PIN, OUTPUT);
  digitalWrite(CONFIG_LED_PIN, LOW);

  Wire.begin(CONFIG_I2C_SDA_PIN, CONFIG_I2C_SCL_PIN);

  uint8_t mpuStatus = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(mpuStatus);

  if (mpuStatus != CONFIG_MPU_STATUS_OK) {
    Serial.println(F("MPU6050 init error, check wiring!"));
    globalImuOk = false;
  } else {
    globalImuOk = true;
    startCalibration();
  }
}

// ================== LOOP ==================

void loop()
{
  if (!globalImuOk) {
    // Optional: error LED blink pattern here
    return;
  }

  // Calibration state
  if (globalCalibState == CALIB_STATE_RUNNING) {
    handleCalibration();
    return;
  }

  // Normal operation after calibration
  if (globalCalibState == CALIB_STATE_DONE) {

    uint32_t currentMicros = micros();
    float    deltaTime     = static_cast<float>(currentMicros - globalLastUpdateMicros)
                             * CONFIG_MICROS_TO_SECONDS;
    globalLastUpdateMicros = currentMicros;

    mpu.update();

    float accelX = mpu.getAccX();
    float accelY = mpu.getAccY();
    float accelZ = mpu.getAccZ();

    float gyroX  = mpu.getGyroX() - globalGyroXOffset;
    float gyroY  = mpu.getGyroY() - globalGyroYOffset;
    float gyroZ  = mpu.getGyroZ() - globalGyroZOffset;

    float accelSquaredY = accelY * accelY;
    float accelSquaredZ = accelZ * accelZ;
    float accelYZNorm   = sqrtf(accelSquaredY + accelSquaredZ);

    float pitchFromAccel = atan2f(-accelX, accelYZNorm) * CONFIG_RAD_TO_DEG;
    float rollFromAccel  = atan2f( accelY, accelZ ) * CONFIG_RAD_TO_DEG;

    float pitchFromGyro  = globalPitchDeg + gyroX * deltaTime;
    float rollFromGyro   = globalRollDeg  + gyroY * deltaTime;
    float yawFromGyro    = globalYawDeg   + gyroZ * deltaTime;

    float complementaryBeta = (1.0f - CONFIG_COMPLEMENTARY_ALPHA);

    globalPitchDeg = CONFIG_COMPLEMENTARY_ALPHA * pitchFromGyro
                   + complementaryBeta          * pitchFromAccel;

    globalRollDeg  = CONFIG_COMPLEMENTARY_ALPHA * rollFromGyro
                   + complementaryBeta          * rollFromAccel;

    globalYawDeg   = yawFromGyro;

    // Wrap yaw into [-180, 180]
    if (globalYawDeg > CONFIG_YAW_WRAP_POS_DEG) {
      globalYawDeg -= CONFIG_FULL_TURN_DEG;
    }
    if (globalYawDeg < CONFIG_YAW_WRAP_NEG_DEG) {
      globalYawDeg += CONFIG_FULL_TURN_DEG;
    }

    uint32_t currentMillis = millis();
    if (currentMillis - globalLastPrintMillis >= CONFIG_PRINT_INTERVAL_MS) {
      globalLastPrintMillis = currentMillis;

      Serial.print(F("PITCH: "));
      Serial.print(globalPitchDeg);
      Serial.print(F("\tROLL: "));
      Serial.print(globalRollDeg);
      Serial.print(F("\tYAW (relative): "));
      Serial.println(globalYawDeg);
      Serial.println(F("====================================================="));
    }
  }
}
