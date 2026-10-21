
#include <WiFi.h>
#include <esp_now.h>

uint8_t ROBOT_MAC[6] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };

#define LINK_MAGIC   0x5A
#define LINK_VERSION 1

#pragma pack(push, 1)
struct LinkPacket {
  uint8_t  magic;
  uint8_t  version;
  uint16_t seq;
  int16_t  x_mm;
  int16_t  y_mm;
  int16_t  heading_cdeg;
  uint8_t  poseValid;
  uint8_t  cmd;
  int16_t  gx_mm;
  int16_t  gy_mm;
  uint8_t  targetId;
  uint8_t  checksum;
};
#pragma pack(pop)

uint8_t buf[sizeof(LinkPacket)];
int     fill = 0;
int     rxState = 0;

void setup() {
  Serial.begin(921600);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("# esp_now_init failed");
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, ROBOT_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("# add_peer failed");
    return;
  }

  Serial.print("# dongle ready, my MAC ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  while (Serial.available()) {
    uint8_t b = Serial.read();

    if (rxState == 0) {
      if (b == 0xAB) rxState = 1;
    } else if (rxState == 1) {
      rxState = (b == 0xCD) ? 2 : 0;
      fill = 0;
    } else {
      buf[fill++] = b;
      if (fill == sizeof(LinkPacket)) {
        esp_now_send(ROBOT_MAC, buf, sizeof(buf));
        rxState = 0;
        fill = 0;
      }
    }
  }
}
