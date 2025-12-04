#include <Arduino.h>
#include "MultiUltrasonic.h"

// ========= USER CONFIG (in .ino) =========

// Common TRIG pin for all ultrasonic sensors
constexpr uint8_t PIN_ULTRASONIC_TRIG = 4;

// Echo pins list – set these as you like
constexpr uint8_t ULTRASONIC_ECHO_PINS[] = {
  13,  // Sensor 0
  18,  // Sensor 1
  19,  // Sensor 2
  23   // Sensor 3
};

constexpr uint8_t ULTRASONIC_SENSOR_COUNT =
    sizeof(ULTRASONIC_ECHO_PINS) / sizeof(ULTRASONIC_ECHO_PINS[0]);

// Max distance (cm) when no echo / too far
constexpr float    MAX_DISTANCE_CM        = 400.0f;
// Time between trigger bursts
constexpr uint32_t BURST_INTERVAL_MS      = 60;
// Print interval
constexpr uint32_t PRINT_INTERVAL_MS      = 200;

void setup() {
  Serial.begin(115200);
  delay(200);

  // Initialize the multi-ultrasonic module with your config
  multiUltrasonicBegin(
    PIN_ULTRASONIC_TRIG,
    ULTRASONIC_ECHO_PINS,
    ULTRASONIC_SENSOR_COUNT,
    MAX_DISTANCE_CM,
    BURST_INTERVAL_MS
  );

  Serial.println("Multi-ultrasonic (config from .ino) started.");
}

void loop() {
  // Non-blocking update
  multiUltrasonicUpdate();

  // Example: print distances periodically
  static uint32_t lastPrintMs = 0;
  uint32_t nowMs = millis();

  if ((nowMs - lastPrintMs) >= PRINT_INTERVAL_MS) {
    lastPrintMs = nowMs;

    uint8_t count = multiUltrasonicSensorCount();
    const float* distances = multiUltrasonicDistances();

    Serial.print("Distances (cm): ");
    for (uint8_t i = 0; i < count; i++) {
      Serial.print("S");
      Serial.print(i);
      Serial.print("=");

      Serial.print(distances[i], 1);  // 1 decimal place
      Serial.print("  ");
    }
    Serial.println();
  }

  // No delay() -> fully non-blocking
}
 