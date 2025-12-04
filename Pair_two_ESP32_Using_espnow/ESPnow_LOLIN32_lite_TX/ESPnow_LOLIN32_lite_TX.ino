#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ================== GLOBAL CONSTANTS ==================

// Serial configuration
const uint32_t SERIAL_BAUD_RATE          = 115200ul;

// LED configuration
const int      TX_LED_PIN                = 2;        // Change to your LED pin if needed
const uint32_t LINK_TIMEOUT_MS           = 2000ul;   // Link considered lost after this time

// Send interval
const uint32_t TX_INTERVAL_MS            = 1000ul;   // Time between packets

// Wi-Fi / ESP-NOW configuration
const wifi_mode_t WIFI_MODE_SETTING      = WIFI_MODE_STA;
const uint8_t     ESPNOW_CHANNEL         = 0u;       // 0 = use current Wi-Fi channel
const bool        ESPNOW_ENCRYPT         = false;

// MAC address configuration
const size_t      MAC_ADDRESS_LENGTH     = 6u;

// Initial time for timers
const uint32_t    INITIAL_TIME_MS        = 0ul;

// Receiver MAC address (your RX ESP32’s MAC)
uint8_t RX_PEER_MAC[MAC_ADDRESS_LENGTH] = {
  0x44, 0x1D, 0x64, 0xF5, 0x7C, 0x5C
};

// ================== PAYLOAD & STATE ==================

typedef struct __attribute__((packed)) {
  uint32_t counter;
} Payload_t;

Payload_t  txPayload              = { 0u };
uint32_t   lastLinkOkTimeMs       = 0u;
uint32_t   lastTxTimeMs           = INITIAL_TIME_MS;

// ================== CALLBACKS ==================

void onDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  (void)info;

  Serial.print("TX send status = ");
  Serial.println(static_cast<int>(status));

  if (status == ESP_NOW_SEND_SUCCESS) {
    lastLinkOkTimeMs = millis();
  }
}

void onDataRecv(const esp_now_recv_info_t* info,
                const uint8_t* incomingData,
                int len) {
  (void)info;

  if (len == static_cast<int>(sizeof(Payload_t))) {
    Payload_t ackPayload;
    memcpy(&ackPayload, incomingData, sizeof(Payload_t));

    Serial.print("TX received ACK, counter = ");
    Serial.println(ackPayload.counter);

    lastLinkOkTimeMs = millis();
  } else {
    Serial.print("TX received data with invalid length = ");
    Serial.println(len);
  }
}

// ================== HELPER FUNCTIONS ==================

void updateLinkLed() {
  const uint32_t nowMs    = millis();
  const bool     linkIsUp = (nowMs - lastLinkOkTimeMs) < LINK_TIMEOUT_MS;

  digitalWrite(TX_LED_PIN, linkIsUp ? HIGH : LOW);
}

void sendPayloadIfDue() {
  const uint32_t nowMs = millis();

  if ((nowMs - lastTxTimeMs) >= TX_INTERVAL_MS) {
    esp_err_t result = esp_now_send(
      RX_PEER_MAC,
      reinterpret_cast<uint8_t*>(&txPayload),
      sizeof(txPayload)
    );

    Serial.print("TX sending, counter = ");
    Serial.println(txPayload.counter);

    if (result != ESP_OK) {
      Serial.print("TX send error, code = ");
      Serial.println(static_cast<int>(result));
    }

    txPayload.counter++;
    lastTxTimeMs = nowMs;
  }
}

// ================== SETUP & LOOP ==================

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(TX_LED_PIN, OUTPUT);
  digitalWrite(TX_LED_PIN, LOW);

  WiFi.mode(WIFI_MODE_SETTING);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) {
      // Stop here if ESP-NOW fails
    }
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, RX_PEER_MAC, MAC_ADDRESS_LENGTH);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = ESPNOW_ENCRYPT;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add RX peer");
    while (true) {
      // Stop here if peer add fails
    }
  }

  lastLinkOkTimeMs = millis();

  Serial.print("This ESP32 STA MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  sendPayloadIfDue();
  updateLinkLed();
}
