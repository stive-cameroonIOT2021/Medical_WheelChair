#ifndef MULTI_ULTRASONIC_H
#define MULTI_ULTRASONIC_H

#include <Arduino.h>

// Maximum number of sensors supported by the library
constexpr uint8_t MULTI_ULTRA_MAX_SENSORS = 8;

// Initialize the module.
//  - trigPin:       common TRIG pin for all sensors
//  - echoPins:      pointer to an array of echo pin numbers
//  - sensorCount:   how many sensors (<= MULTI_ULTRA_MAX_SENSORS)
//  - maxDistanceCm: distance used when echo is missing / too far
//  - burstIntervalMs: time between TRIG bursts
void multiUltrasonicBegin(uint8_t trigPin,
                          const uint8_t* echoPins,
                          uint8_t sensorCount,
                          float maxDistanceCm     = 400.0f,
                          uint32_t burstIntervalMs = 60);

// Call this frequently in loop(); non-blocking.
void multiUltrasonicUpdate();

// Number of active sensors.
uint8_t multiUltrasonicSensorCount();

// Get read-only pointer to internal distances array (cm).
// Only the first multiUltrasonicSensorCount() entries are valid.
const float* multiUltrasonicDistances();

// Convenience: get distance for a single sensor (cm).
// Returns maxDistance if index invalid.
float multiUltrasonicGetDistance(uint8_t index);

// Current configured "no echo / too far" distance (cm).
float multiUltrasonicGetMaxDistance();

#endif // MULTI_ULTRASONIC_H
