#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_mac.h>   // for esp_read_mac & ESP_MAC_WIFI_STA

// ================== CONFIG ==================

const uint8_t CONFIG_LED_PIN           = 2;      // Onboard LED
const uint32_t CONFIG_LINK_TIMEOUT_MS  = 1000;   // ms

// TX MAC (peer)
uint8_t CONFIG_TX_MAC[6] = {0xC0, 0x49, 0xEF, 0xF1, 0x5A, 0xB8};

// ACK payload
const char CONFIG_ACK_TEXT[] = "ACK";

// Packet structure (must match TX!)
struct AnglePacket {
  float pitchDeg;
  float rollDeg;
  float yawDeg;
};

// ================== GLOBAL STATE ==================

bool     globalLinkActive    = false;
uint32_t globalLastRxMillis  = 0;

// ================== HELPERS ==================

void printMac(const uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    if (i > 0) Serial.print(":");
    if (mac[i] < 0x10) Serial.print("0");
    Serial.print(mac[i], HEX);
  }
}

// ======== ESP-NOW CALLBACKS ========

void onEspNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info; // unused
  Serial.print(F("RX send status (ACK): "));
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? F("Success") : F("Fail"));
}

void onEspNowRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int len) {
  Serial.print(F("RX received from: "));
  printMac(recvInfo->src_addr);
  Serial.print(F(" | Len: "));
  Serial.println(len);

  globalLastRxMillis = millis();
  globalLinkActive   = true;
  Serial.println(F("RX: packet received → link active"));

  // We expect angle packets with exact size
  if (len == sizeof(AnglePacket)) {
    AnglePacket pkt;
    memcpy(&pkt, data, sizeof(AnglePacket));

    Serial.print(F("Angles -> Pitch: "));
    Serial.print(pkt.pitchDeg);
    Serial.print(F("  Roll: "));
    Serial.print(pkt.rollDeg);
    Serial.print(F("  Yaw: "));
    Serial.println(pkt.yawDeg);
  } else {
    Serial.print(F("Payload (raw): "));
    for (int i = 0; i < len; i++) {
      Serial.print((char)data[i]);
    }
    Serial.println();
  }

  // Send ACK back to TX
  esp_err_t result = esp_now_send(
    CONFIG_TX_MAC,
    reinterpret_cast<const uint8_t*>(CONFIG_ACK_TEXT),
    3   // "ACK"
  );

  if (result == ESP_OK) {
    Serial.println(F("RX: ACK sent"));
  } else {
    Serial.print(F("RX: Error sending ACK, code: "));
    Serial.println(result);
  }
}

// ================== SETUP ==================

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(CONFIG_LED_PIN, OUTPUT);
  digitalWrite(CONFIG_LED_PIN, LOW);

  WiFi.mode(WIFI_STA);

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

  esp_now_register_send_cb(onEspNowSent);
  esp_now_register_recv_cb(onEspNowRecv);

  // Add TX as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, CONFIG_TX_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

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
  uint32_t now = millis();

  if (globalLinkActive && (now - globalLastRxMillis > CONFIG_LINK_TIMEOUT_MS)) {
    globalLinkActive = false;
    Serial.println(F("RX: link timeout → LED OFF"));
  }

  digitalWrite(CONFIG_LED_PIN, globalLinkActive ? HIGH : LOW);
}
