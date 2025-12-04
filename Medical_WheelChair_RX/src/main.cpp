#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <math.h>        // fabsf, isnan, lroundf
#include <esp_now.h>
#include <WiFi.h>
#include <esp_mac.h>     // esp_read_mac & ESP_MAC_WIFI_STA
#include "multiUltrasonic.h"  // Multi-ultrasonic sensor module


/*================== Buzzer config ==================*/
#define BUZZER_PIN 5      // Change to your chosen safe pin
bool passiveBuzzer = false;   // true = passive (piezo), false = active (with internal oscillator)

/*================== Buzzer timing ==================*/
const uint16_t BUZZER_FREQ     = 2000;  // Hz (for passive buzzer)
const uint8_t  BUZZER_CHANNEL  = 0;     // LEDC channel for tone
const uint8_t  BUZZER_RES_BITS = 8;     // not very important for tone

// State
bool     buzzerIsOn      = false;
uint32_t buzzerOffTimeMs = 0;

// Optional: simple debounce between beeps
uint32_t lastBeepRequestMs = 0;
uint32_t minBeepGapMs      = 50;  // ignore new beeps closer than this


/*================== Ultrasonic sensor config ==================*/
// Common TRIG pin for all ultrasonic sensors
constexpr uint8_t PIN_ULTRASONIC_TRIG = 4;

// Echo pins list – set these as you like
constexpr uint8_t ULTRASONIC_ECHO_PINS[] = {
  13,  // Sensor 0 Back sensor
  18,  // Sensor 1 Right sensor
  19,  // Sensor 2 Left sensor
  23   // Sensor 3 Front sensor
};

constexpr uint8_t ULTRASONIC_SENSOR_COUNT =
    sizeof(ULTRASONIC_ECHO_PINS) / sizeof(ULTRASONIC_ECHO_PINS[0]);

// Max distance (cm) when no echo / too far
constexpr float    MAX_DISTANCE_CM        = 400.0f;
// Time between trigger bursts
constexpr uint32_t BURST_INTERVAL_MS      = 60;
// Print interval
constexpr uint32_t PRINT_INTERVAL_MS      = 200;


// Index enum for readability
enum : uint8_t {
  IDX_BACK  = 0,
  IDX_RIGHT = 1,
  IDX_LEFT  = 2,
  IDX_FRONT = 3
};

// Obstacle distance thresholds (cm)
constexpr float FRONT_STOP_CM = 30.0f;  // stop forward if front < 30 cm
constexpr float BACK_STOP_CM  = 30.0f;  // stop backward if back  < 30 cm
constexpr float SIDE_STOP_CM  = 25.0f;  // stop turning toward side if < 25 cm

// Global copy of last measured distances
float g_ultraDistances[ULTRASONIC_SENSOR_COUNT] = {
  MAX_DISTANCE_CM, MAX_DISTANCE_CM,
  MAX_DISTANCE_CM, MAX_DISTANCE_CM
};
/*--------------------------------------------------------------------------*/

// ================= Relays =================
// These four pins drive external relays that handle motor direction.
const int IN1_PIN = 12;
const int IN2_PIN = 14;
const int IN3_PIN = 27;
const int IN4_PIN = 26;

// Relay direction state:
// true  -> FORWARD  (IN1=HIGH, IN2=HIGH, IN3=LOW,  IN4=LOW)
// false -> BACKWARD (IN1=LOW,  IN2=LOW,  IN3=HIGH, IN4=HIGH)
bool relayForward = true;

// ================= I2C / DAC config =================
// One MCP4725 on each I2C bus (Wire / Wire1)
constexpr uint8_t  DAC_ADDR   = 0x60;   // MCP4725 address (both)

// I2C buses
constexpr int SDA1_PIN   = 21;          // Bus 1: DAC1
constexpr int SCL1_PIN   = 22;
constexpr int SDA2_PIN   = 16;          // Bus 2: DAC2
constexpr int SCL2_PIN   = 17;

constexpr uint32_t I2C_CLOCK  = 400000;

constexpr uint8_t  DAC_BITS     = 12;
constexpr uint16_t DAC_MAX_CODE = (1u << DAC_BITS) - 1u;

// ------- Global voltage range (shared by both DACs) -------
// These define the commanded voltage range mapped from PWM.
float V_MIN = 0.0f;
float V_MAX = 5.0f;

// ------- Real DAC supply (set VREF to your measured value) -------
// This should be your MCP4725 VCC measured with a multimeter.
float VREF = 5.000f;  // <- put your measured MCP4725 VCC here

// ------- Calibration (shared): measured = CAL_A * commanded + CAL_B -------
// You can tune these if your real output is off.
float CAL_A = 1.000f; // gain
float CAL_B = 0.000f; // offset (volts)

// ------- Dynamics / pacing (shared) -------
// SLEW_V_PER_S: max change in voltage per second (0 => instant step)
float    SLEW_V_PER_S       = 2.0f;
// UPDATE_INTERVAL_US: minimum interval between actual I2C writes
uint32_t UPDATE_INTERVAL_US = 2000u;       // µs between I2C writes

// ================== ESP-NOW / ROBOT CONFIG ==================

// Status LED
constexpr uint8_t  CONFIG_LED_PIN         = 2;        // Onboard LED
constexpr uint32_t CONFIG_LINK_TIMEOUT_MS = 1000;     // ms without packets => link lost

// TX MAC (peer) - MUST match your TX ESP32 STA MAC
uint8_t CONFIG_TX_MAC[6] = {0xC0, 0x49, 0xEF, 0xF1, 0x5A, 0xB8};

// ACK payload
constexpr char CONFIG_ACK_TEXT[] = "ACK";

// Packet structure (MUST MATCH TX!)
struct SpeedPacket {
  uint16_t pwmL;   // 0..1023 magnitude
  uint16_t pwmR;   // 0..1023 magnitude
  int8_t   dir;    // -1 = backward, 0 = stop, 1 = forward
};

// --------- PWM CONFIG (10-bit resolution) ---------
constexpr int SPEED_BITS        = 10;
constexpr int SPEED_MAX         = (1 << SPEED_BITS) - 1;  // 1023
constexpr int SPEED_STOP        = 0;

// Global PWM limiter [0..1023] (can be changed later)
int globalPwmLimit = 560;       // limit to keep voltage & current safe

// ================== GLOBAL STATE ==================

bool     globalLinkActive    = false;
uint32_t globalLastRxMillis  = 0;

// PWM sent to DACs (after limiter)
int globalPwmL = 0;   // [0..1023]
int globalPwmR = 0;   // [0..1023]

// ================== HELPERS ==================

void buzzerOn() {
  if (buzzerIsOn) return;

  if (passiveBuzzer) {
    // Passive: use hardware PWM/tone
    ledcWriteTone(BUZZER_CHANNEL, BUZZER_FREQ);
  } else {
    // Active: just drive pin HIGH
    digitalWrite(BUZZER_PIN, HIGH);
  }
  buzzerIsOn = true;
}

void buzzerOff() {
  if (!buzzerIsOn) return;

  if (passiveBuzzer) {
    ledcWriteTone(BUZZER_CHANNEL, 0);  // stop tone
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
  buzzerIsOn = false;
}

void buzzerBeep(uint32_t durationMs) {
  uint32_t now = millis();

  // Optional: ignore too-frequent requests
  if (now - lastBeepRequestMs < minBeepGapMs) return;
  lastBeepRequestMs = now;

  buzzerOn();
  buzzerOffTimeMs = now + durationMs;
}

void buzzerUpdate() {
  uint32_t now = millis();
  if (buzzerIsOn && now >= buzzerOffTimeMs) {
    buzzerOff();
  }
}


inline void printMac(const uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    if (i > 0) Serial.print(':');
    if (mac[i] < 0x10) Serial.print('0');
    Serial.print(mac[i], HEX);
  }
}

inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

inline uint32_t elapsedUs(uint32_t start, uint32_t now) {
  return static_cast<uint32_t>(now - start);
}

// Apply global PWM limiter [0..globalPwmLimit]
inline int applyPwmLimit(int pwm) {
  if (pwm < 0) pwm = 0;
  if (pwm > globalPwmLimit) pwm = globalPwmLimit;
  return pwm;
}

// Map speed [0..SPEED_MAX] to voltage based on current V_MIN/V_MAX
float speedToVoltage(float speed) {
  speed = clampf(speed, 0.0f, static_cast<float>(SPEED_MAX));
  const float ratio = speed / static_cast<float>(SPEED_MAX); // 0.0 .. 1.0
  return V_MIN + ratio * (V_MAX - V_MIN);
}

// ================= MCP4725 Controller CLASS =================
// Non-blocking DAC controller with slew rate limiting.
class MCP4725Controller {
public:
  bool begin(TwoWire *wire, uint8_t addr) {
    _wire = wire;
    _addr = addr;
    if (!_dac.begin(_addr, _wire)) return false;

    const uint32_t now = micros();
    _lastUpdateUs = _lastTickUs = now;

    // Smallest meaningful step in volts (~1 LSB at VREF)
    _minStepVolts = VREF / static_cast<float>(DAC_MAX_CODE);

    _currentV = clampf(0.0f, V_MIN, V_MAX);
    _targetV  = _currentV;
    writeVoltage(_currentV);
    return true;
  }

  // Non-blocking setter: only updates the target; real output changes in update()
  void setTarget(float v) {
    _targetV = clampf(v, V_MIN, V_MAX);
  }

  // Must be called regularly; handles slew + I2C writes
  void update() {
    const uint32_t now = micros();
    if (elapsedUs(_lastTickUs, now) < UPDATE_INTERVAL_US) return;
    _lastTickUs = now;

    float next = _currentV;

    if (SLEW_V_PER_S <= 0.0f) {
      // step instantly
      next = _targetV;
    } else {
      const float dt      = elapsedUs(_lastUpdateUs, now) / 1e6f;
      const float maxStep = SLEW_V_PER_S * dt;
      const float err     = _targetV - _currentV;

      if (err >  maxStep) next = _currentV + maxStep;
      else if (err < -maxStep) next = _currentV - maxStep;
      else                     next = _targetV;
    }

    next = clampf(next, V_MIN, V_MAX);

    // Only write if change is at least ~1 LSB
    if (fabsf(next - _lastWrittenV) >= _minStepVolts) {
      writeVoltage(next);
      _lastWrittenV = next;
    }

    _currentV     = next;
    _lastUpdateUs = now;
  }

  float current() const { return _currentV; }
  float target()  const { return _targetV;  }

private:
  void writeVoltage(float vLimited) {
    // vLimited is already within [V_MIN..V_MAX]
    float commanded = (vLimited - CAL_B) / CAL_A;      // undo calibration
    commanded       = clampf(commanded, 0.0f, VREF);   // DAC can't exceed VREF

    uint16_t code = static_cast<uint16_t>(
      lroundf((commanded / VREF) * DAC_MAX_CODE)
    );
    if (code > DAC_MAX_CODE) code = DAC_MAX_CODE;

    _dac.setVoltage(code, false);
  }

  Adafruit_MCP4725 _dac;
  TwoWire*  _wire          = nullptr;
  uint8_t   _addr          = DAC_ADDR;
  float     _currentV      = 0.0f;
  float     _targetV       = 0.0f;
  float     _lastWrittenV  = -1000.0f;
  float     _minStepVolts  = 0.0f;
  uint32_t  _lastUpdateUs  = 0;
  uint32_t  _lastTickUs    = 0;
};

// ================= Instances & globals =================
MCP4725Controller dacs[2];   // dacs[0] on Wire, dacs[1] on Wire1

// ================= Relay helper =================
// Non-blocking: just writes the pins, no delay.
void applyRelayState() {
  const uint8_t in1Level = relayForward ? HIGH : LOW;
  const uint8_t in2Level = relayForward ? HIGH : LOW;
  const uint8_t in3Level = relayForward ? LOW  : HIGH;
  const uint8_t in4Level = relayForward ? LOW  : HIGH;

  digitalWrite(IN1_PIN, in1Level);
  digitalWrite(IN2_PIN, in2Level);
  digitalWrite(IN3_PIN, in3Level);
  digitalWrite(IN4_PIN, in4Level);

  /* Serial.print(F("Relays set to: "));
  Serial.println(relayForward ?
    F("FORWARD (IN1/IN2 HIGH, IN3/IN4 LOW)") :
    F("BACKWARD (IN1/IN2 LOW, IN3/IN4 HIGH)")
  ); */
}

// ======== ESP-NOW CALLBACKS (classic API – matches your framework) ========

// Called after we send something with esp_now_send
// Classic signature: mac_addr + status
void onEspNowSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  (void)mac_addr;
  (void)status;
  // Optional debug:
  // Serial.print(F("ACK status: "));
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? F("OK") : F("FAIL"));
}

// Called when a packet is received
// Classic signature: mac_addr + data + len
void onEspNowRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  (void)mac_addr; // not used, but available if you want to filter by sender

  // Mark link as active
  globalLastRxMillis = millis();
  globalLinkActive   = true;

  // We expect SpeedPacket with exact size
  if (len == static_cast<int>(sizeof(SpeedPacket))) {
    SpeedPacket pkt;
    memcpy(&pkt, data, sizeof(SpeedPacket));

    // Sanitize input PWM
    int pwmL = static_cast<int>(pkt.pwmL);
    int pwmR = static_cast<int>(pkt.pwmR);

    if (pwmL < 0) pwmL = 0;
    if (pwmR < 0) pwmR = 0;
    if (pwmL > SPEED_MAX) pwmL = SPEED_MAX;
    if (pwmR > SPEED_MAX) pwmR = SPEED_MAX;

    // Apply limiter
    pwmL = applyPwmLimit(pwmL);
    pwmR = applyPwmLimit(pwmR);

    // Direction:
    // dir > 0 → forward
    // dir < 0 → backward
    // dir == 0 → STOP (all PWM = 0)
    if (pkt.dir > 0) {
      relayForward = true;
    } else if (pkt.dir < 0) {
      relayForward = false;
    }

    if (pkt.dir == 0) {
      // STOP, ignore pwm values, zero everything
      globalPwmL = SPEED_STOP;
      globalPwmR = SPEED_STOP;
    } else {
      globalPwmL = pwmL;
      globalPwmR = pwmR;
    }

    applyRelayState();

    // Debug (uncomment if needed)
    /*
    Serial.print(F("RX pkt: dir="));
    Serial.print((int)pkt.dir);
    Serial.print(F(" pwmL="));
    Serial.print(globalPwmL);
    Serial.print(F(" pwmR="));
    Serial.println(globalPwmR);
    */

  } else {
    Serial.print(F("Unexpected payload size ("));
    Serial.print(len);
    Serial.print(F("), expected "));
    Serial.println(sizeof(SpeedPacket));
    Serial.println(F("→ STOP for safety"));

    // STOP everything on bad packet
    globalPwmL = SPEED_STOP;
    globalPwmR = SPEED_STOP;
  }

  // Send ACK back to TX (non-blocking; ESP-NOW send is async)
  const esp_err_t result = esp_now_send(
    CONFIG_TX_MAC,
    reinterpret_cast<const uint8_t*>(CONFIG_ACK_TEXT),
    sizeof(CONFIG_ACK_TEXT) - 1   // "ACK" (3 bytes, no null)
  );

  if (result != ESP_OK) {
    Serial.print(F("RX: Error sending ACK, code: "));
    Serial.println(result);
  }
}

// ================== SETUP ==================

void setup() {
  Serial.begin(115200);

  if (passiveBuzzer) {
    // Setup LEDC for passive buzzer
    ledcSetup(BUZZER_CHANNEL, BUZZER_FREQ, BUZZER_RES_BITS);
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  } else {
    // Active buzzer is just a digital output
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
  }

  buzzerOff();
  // no delay() here to keep 100% non-blocking beyond hardware init

  // ========== Relays ==========
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  relayForward = true;         // default direction
  applyRelayState();

    // Initialize the multi-ultrasonic module with your config
  multiUltrasonicBegin(
    PIN_ULTRASONIC_TRIG,
    ULTRASONIC_ECHO_PINS,
    ULTRASONIC_SENSOR_COUNT,
    MAX_DISTANCE_CM,
    BURST_INTERVAL_MS
  );

  Serial.println("Multi-ultrasonic (config from .ino) started.");

  // ========== I2C & DAC init ==========
  Wire.begin(SDA1_PIN, SCL1_PIN);   // DAC1 on Wire
  Wire.setClock(I2C_CLOCK);

  Wire1.begin(SDA2_PIN, SCL2_PIN);  // DAC2 on Wire1
  Wire1.setClock(I2C_CLOCK);

  // Ensure range is sane vs. VREF
  V_MIN = clampf(V_MIN, 0.0f, VREF);
  V_MAX = clampf(V_MAX, V_MIN, VREF);

  const bool ok1 = dacs[0].begin(&Wire,  DAC_ADDR);
  const bool ok2 = dacs[1].begin(&Wire1, DAC_ADDR);

  if (!ok1) Serial.println(F("DAC1 (MCP4725) not found on Wire. Check wiring."));
  if (!ok2) Serial.println(F("DAC2 (MCP4725) not found on Wire1. Check wiring."));

  if (ok1 || ok2) {
    Serial.println(F("MCP4725 dual-DAC ready."));
  } else {
    Serial.println(F("No DACs found. Check I2C pins and power."));
  }

  // ========== ESP-NOW / WiFi ==========
  pinMode(CONFIG_LED_PIN, OUTPUT);
  digitalWrite(CONFIG_LED_PIN, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();   // no blocking reconnection attempts

  // Print this ESP32 MAC (RX)
  uint8_t rxMac[6];
  if (esp_read_mac(rxMac, ESP_MAC_WIFI_STA) == ESP_OK) {
    Serial.print(F("RX ESP32 STA MAC: "));
    printMac(rxMac);
    Serial.println();
  } else {
    Serial.println(F("Failed to read RX MAC"));
  }

  // ESP-NOW init
  if (esp_now_init() != ESP_OK) {
    Serial.println(F("RX: Error initializing ESP-NOW"));
    return;
  }

  // Register callbacks (classic API)
  esp_now_register_send_cb(onEspNowSent);
  esp_now_register_recv_cb(onEspNowRecv);

  // Add TX as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, CONFIG_TX_MAC, 6);
  peerInfo.channel = 0;      // current WiFi channel
  peerInfo.encrypt = false;  // no encryption

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println(F("RX: Failed to add TX peer"));
    return;
  }

  Serial.print(F("RX: added TX peer: "));
  printMac(CONFIG_TX_MAC);
  Serial.println();
}

// ================== LOOP ==================

void loop() {
  const uint32_t now = millis();
  static uint32_t lastDemoBeep = 0;

  // FAST non-blocking ultrasonic update
  multiUltrasonicUpdate();

  // Copy latest distances into global array
  const uint8_t count = multiUltrasonicSensorCount();
  const float* distances = multiUltrasonicDistances();
  if (distances != nullptr) {
    const uint8_t n = (count < ULTRASONIC_SENSOR_COUNT) ?
                      count : ULTRASONIC_SENSOR_COUNT;
    for (uint8_t i = 0; i < n; ++i) {
      g_ultraDistances[i] = distances[i];
    }
  }

  // Optional debug for distances
  
  static uint32_t lastPrintUltra = 0;
  if (now - lastPrintUltra >= PRINT_INTERVAL_MS) {
    lastPrintUltra = now;
    Serial.print(F("Ultrasonic (cm) B:"));
    Serial.print(g_ultraDistances[IDX_BACK], 1);
    Serial.print(F(" R:"));
    Serial.print(g_ultraDistances[IDX_RIGHT], 1);
    Serial.print(F(" L:"));
    Serial.print(g_ultraDistances[IDX_LEFT], 1);
    Serial.print(F(" F:"));
    Serial.println(g_ultraDistances[IDX_FRONT], 1);
  }
 

  // Link timeout handling (non-blocking)
  if (globalLinkActive && (now - globalLastRxMillis > CONFIG_LINK_TIMEOUT_MS)) {
    globalLinkActive = false;
    Serial.println(F("RX: link timeout → STOP"));

    // Safety: stop robot
    globalPwmL = SPEED_STOP;
    globalPwmR = SPEED_STOP;
  }

  if(g_ultraDistances[IDX_FRONT] < FRONT_STOP_CM) {
    // Obstacle in front → stop forward motion
    if(relayForward && globalPwmL > 0) {
      globalPwmL = SPEED_STOP;
    }
    if(relayForward && globalPwmR > 0) {
      globalPwmR = SPEED_STOP;
    }
  if (now - lastDemoBeep >= 5000) {
    lastDemoBeep = now;
    buzzerBeep(150);    // 150 ms beep
  }
  }

  if(g_ultraDistances[IDX_BACK] < BACK_STOP_CM) {
    // Obstacle in back → stop backward motion
    if(!relayForward && globalPwmL > 0) {
      globalPwmL = SPEED_STOP;
    }
    
    if(!relayForward && globalPwmR > 0) {
      globalPwmR = SPEED_STOP;
    }
  }

  if(g_ultraDistances[IDX_LEFT] < SIDE_STOP_CM) {
    // Obstacle on left → stop left turn (i.e., right wheel forward)
    if(relayForward && globalPwmR > 0) {
      globalPwmR = SPEED_STOP;
    }
    if(!relayForward && globalPwmL > 0) {
      globalPwmL = SPEED_STOP;
    }
  }

  if(g_ultraDistances[IDX_RIGHT] < SIDE_STOP_CM) {
    // Obstacle on right → stop right turn (i.e., left wheel forward)
    if(relayForward && globalPwmL > 0) {
      globalPwmL = SPEED_STOP;
    }
    if(!relayForward && globalPwmR > 0) {
      globalPwmR = SPEED_STOP;
    }
  }

  // LED ON when link active, OFF otherwise
  digitalWrite(CONFIG_LED_PIN, globalLinkActive ? HIGH : LOW);

  // Convert current PWM to voltages for each DAC
  const float v1 = speedToVoltage(static_cast<float>(globalPwmL));
  const float v2 = speedToVoltage(static_cast<float>(globalPwmR));

  // Optional periodic debug
  /*
  static uint32_t lastPrint = 0;
  if (now - lastPrint >= 500) {
    lastPrint = now;
    Serial.print(F("PWM L: "));
    Serial.print(globalPwmL);
    Serial.print(F(" -> Vout1: "));
    Serial.print(v1, 3);
    Serial.print(F(" V | PWM R: "));
    Serial.print(globalPwmR);
    Serial.print(F(" -> Vout2: "));
    Serial.print(v2, 3);
    Serial.println(F(" V"));
  }
  */

  // MUST be called frequently
  buzzerUpdate();

  // Update DAC targets & slew (non-blocking)
  dacs[0].setTarget(v1);
  dacs[1].setTarget(v2);

  dacs[0].update();
  dacs[1].update();
}
