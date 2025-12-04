#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ================== GLOBAL CONSTANTS ==================

const uint32_t SERIAL_BAUD_RATE          = 115200ul;

const int      RX_LED_PIN                = 2;        // change here if needed
const uint32_t LINK_TIMEOUT_MS           = 2000ul;

const wifi_mode_t WIFI_MODE_SETTING      = WIFI_MODE_STA;

const uint8_t  ESPNOW_CHANNEL            = 0u;
const bool     ESPNOW_ENCRYPT            = false;

const size_t   MAC_ADDRESS_LENGTH        = 6u;
const uint32_t INITIAL_TIME_MS           = 0u;

// Sender MAC (your TX board)
uint8_t TX_PEER_MAC[MAC_ADDRESS_LENGTH] = {
  0xC0, 0x49, 0xEF, 0xF1, 0x5A, 0xB8
};

// ================== PAYLOAD & STATE ==================

typedef struct __attribute__((packed)) {
  uint32_t counter;
} Payload_t;

Payload_t  rxLastPayload      = { 0u };
Payload_t  replyPayload       = { 0u };
uint32_t   lastLinkOkTimeMs   = 0u;

// ================== CALLBACKS ==================

void onDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  (void)info;
  (void)status;
  // we consider link alive when receiving, not on send
}

void onDataRecv(const esp_now_recv_info_t* info,
                const uint8_t* incomingData,
                int len) {
  (void)info;

  if (len == static_cast<int>(sizeof(Payload_t))) {
    memcpy(&rxLastPayload, incomingData, sizeof(Payload_t));
    lastLinkOkTimeMs = millis();

    Serial.print("RX got counter = ");
    Serial.println(rxLastPayload.counter);

    replyPayload.counter = rxLastPayload.counter;

    esp_err_t result = esp_now_send(
      TX_PEER_MAC,
      reinterpret_cast<uint8_t*>(&replyPayload),
      sizeof(replyPayload)
    );

    if (result == ESP_OK) {
      Serial.print("Reply sent, counter = ");
      Serial.println(replyPayload.counter);
    } else {
      Serial.print("Reply send error, code = ");
      Serial.println(static_cast<int>(result));
    }
  } else {
    Serial.print("RX got invalid length = ");
    Serial.println(len);
  }
}

// ================== HELPERS ==================

void updateLinkLed() {
  const uint32_t nowMs    = millis();
  const bool     linkIsUp = (nowMs - lastLinkOkTimeMs) < LINK_TIMEOUT_MS;

  digitalWrite(RX_LED_PIN, linkIsUp ? HIGH : LOW);
}

// ================== SETUP & LOOP ==================

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(RX_LED_PIN, OUTPUT);
  digitalWrite(RX_LED_PIN, LOW);

  WiFi.mode(WIFI_MODE_SETTING);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) {
      // stop here
    }
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, TX_PEER_MAC, MAC_ADDRESS_LENGTH);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = ESPNOW_ENCRYPT;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add TX peer");
    while (true) {
      // stop here
    }
  }

  lastLinkOkTimeMs = millis();

  Serial.print("This ESP32 STA MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  updateLinkLed();
}
