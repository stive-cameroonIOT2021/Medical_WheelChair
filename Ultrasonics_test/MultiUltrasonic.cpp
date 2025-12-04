#include "MultiUltrasonic.h"

namespace
{
    // ---- Constants ----
    constexpr float    SPEED_OF_SOUND_CM_PER_US = 0.0343f;  // ~343 m/s -> 0.0343 cm/µs

    // Internal storage (max sensors)
    uint8_t  g_trigPin = 0;
    uint8_t  g_sensorCount = 0;
    uint8_t  g_echoPins[MULTI_ULTRA_MAX_SENSORS] = {0};

    // Configurable parameters
    float    g_maxDistanceCm     = 400.0f;
    uint32_t g_burstIntervalMs   = 60;
    uint32_t g_maxEchoTimeUs     = 0;      // derived from maxDistance

    // Timing and state
    volatile uint32_t g_echoStartUs[MULTI_ULTRA_MAX_SENSORS] = {0};
    volatile uint32_t g_echoEndUs[MULTI_ULTRA_MAX_SENSORS]   = {0};
    volatile bool     g_echoRisingSeen[MULTI_ULTRA_MAX_SENSORS]   = {false};
    volatile bool     g_measurementReady[MULTI_ULTRA_MAX_SENSORS] = {false};

    float g_distanceCm[MULTI_ULTRA_MAX_SENSORS] = {0.0f};

    enum class TrigState : uint8_t {
        Idle,
        PulseHigh
    };

    TrigState  g_trigState        = TrigState::Idle;
    uint32_t   g_trigChangeTimeUs = 0;
    uint32_t   g_lastBurstTimeMs  = 0;

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

    // ======= ISR core =======
    void IRAM_ATTR handleEchoISR(uint8_t sensorIndex) {
        if (sensorIndex >= g_sensorCount) {
            return;
        }

        uint8_t pin = g_echoPins[sensorIndex];
        int level = digitalRead(pin);
        uint32_t nowUs = micros();

        if (level == HIGH) {
            // Rising edge: start of echo
            g_echoStartUs[sensorIndex] = nowUs;
            g_echoRisingSeen[sensorIndex] = true;
        } else {
            // Falling edge: end of echo
            if (g_echoRisingSeen[sensorIndex]) {
                g_echoEndUs[sensorIndex] = nowUs;
                g_measurementReady[sensorIndex] = true;
                g_echoRisingSeen[sensorIndex] = false;
            }
        }
    }

    // ISR wrappers (support up to MULTI_ULTRA_MAX_SENSORS)
    void IRAM_ATTR echoISR0() { handleEchoISR(0); }
    void IRAM_ATTR echoISR1() { handleEchoISR(1); }
    void IRAM_ATTR echoISR2() { handleEchoISR(2); }
    void IRAM_ATTR echoISR3() { handleEchoISR(3); }
    void IRAM_ATTR echoISR4() { handleEchoISR(4); }
    void IRAM_ATTR echoISR5() { handleEchoISR(5); }
    void IRAM_ATTR echoISR6() { handleEchoISR(6); }
    void IRAM_ATTR echoISR7() { handleEchoISR(7); }

    using ISRFunc = void (*)();
    ISRFunc g_echoIsrFuncs[MULTI_ULTRA_MAX_SENSORS] = {
        echoISR0,
        echoISR1,
        echoISR2,
        echoISR3,
        echoISR4,
        echoISR5,
        echoISR6,
        echoISR7
    };

    // ======= TRIG state machine =======
    void updateTrigStateInternal() {
        if (g_sensorCount == 0) {
            return; // not configured
        }

        uint32_t nowMs = millis();
        uint32_t nowUs = micros();

        switch (g_trigState) {
            case TrigState::Idle:
                if ((nowMs - g_lastBurstTimeMs) >= g_burstIntervalMs) {
                    digitalWrite(g_trigPin, HIGH);
                    g_trigChangeTimeUs = nowUs;
                    g_trigState = TrigState::PulseHigh;
                }
                break;

            case TrigState::PulseHigh:
                if ((nowUs - g_trigChangeTimeUs) >= 10) {  // 10 µs pulse
                    digitalWrite(g_trigPin, LOW);
                    g_trigState = TrigState::Idle;
                    g_lastBurstTimeMs = nowMs;
                }
                break;
        }
    }

    // ======= Measurement processing =======
    void processMeasurementsInternal() {
        if (g_sensorCount == 0) return;

        for (uint8_t i = 0; i < g_sensorCount; i++) {
            bool     ready   = false;
            uint32_t startUs = 0;
            uint32_t endUs   = 0;

            noInterrupts();
            if (g_measurementReady[i]) {
                g_measurementReady[i] = false;
                ready   = true;
                startUs = g_echoStartUs[i];
                endUs   = g_echoEndUs[i];
            }
            interrupts();

            if (!ready) continue;

            uint32_t durationUs = (endUs >= startUs)
                                  ? (endUs - startUs)
                                  : (0xFFFFFFFFu - startUs + endUs + 1u); // micros() overflow

            if (durationUs == 0 || durationUs > g_maxEchoTimeUs) {
                g_distanceCm[i] = g_maxDistanceCm;
            } else {
                float distance = (durationUs * SPEED_OF_SOUND_CM_PER_US) / 2.0f;
                g_distanceCm[i] = distance;
            }
        }
    }

} // namespace

// ======= Public API =======

void multiUltrasonicBegin(uint8_t trigPin,
                          const uint8_t* echoPins,
                          uint8_t sensorCount,
                          float maxDistanceCm,
                          uint32_t burstIntervalMs)
{
    g_trigPin = trigPin;

    if (sensorCount > MULTI_ULTRA_MAX_SENSORS) {
        sensorCount = MULTI_ULTRA_MAX_SENSORS;
    }
    g_sensorCount = sensorCount;

    g_maxDistanceCm   = maxDistanceCm;
    g_burstIntervalMs = burstIntervalMs;
    g_maxEchoTimeUs   = static_cast<uint32_t>(
        (g_maxDistanceCm * 2.0f) / SPEED_OF_SOUND_CM_PER_US
    );

    pinMode(g_trigPin, OUTPUT);
    digitalWrite(g_trigPin, LOW);

    for (uint8_t i = 0; i < g_sensorCount; i++) {
        g_echoPins[i] = echoPins[i];
        pinMode(g_echoPins[i], INPUT);
        attachInterrupt(
            digitalPinToInterrupt(g_echoPins[i]),
            g_echoIsrFuncs[i],
            CHANGE
        );
        g_distanceCm[i] = g_maxDistanceCm; // init as "no obstacle"
    }

    g_trigState       = TrigState::Idle;
    g_lastBurstTimeMs = millis();
}

void multiUltrasonicUpdate() {
    updateTrigStateInternal();
    processMeasurementsInternal();
}

uint8_t multiUltrasonicSensorCount() {
    return g_sensorCount;
}

const float* multiUltrasonicDistances() {
    return g_distanceCm;
}

float multiUltrasonicGetDistance(uint8_t index) {
    if (index >= g_sensorCount) {
        return g_maxDistanceCm;
    }
    return g_distanceCm[index];
}

float multiUltrasonicGetMaxDistance() {
    return g_maxDistanceCm;
}
