#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

const char *ROBOT_MAC_STR = "A4:F0:0F:EF:BB:F0";

#define STATUS_LED 2
#define ESPNOW_CHANNEL 1

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

uint8_t robotMac[6];
uint8_t buf[sizeof(LinkPacket)];
int     fill = 0;
int     rxState = 0;
bool    peerReady = false;

uint32_t sent = 0;
uint32_t failed = 0;
unsigned long lastPacket = 0;
unsigned long lastReport = 0;

bool ParseMac(const char *s, uint8_t *out) {
  int v[6];
  if (sscanf(s, "%x:%x:%x:%x:%x:%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) return false;
  for (int i = 0; i < 6; i++) out[i] = (uint8_t)v[i];
  return true;
}

void OnSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) sent++;
  else failed++;
}

void setup() {
  Serial.begin(921600);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.println();
  Serial.print("# dongle MAC  ");
  Serial.println(WiFi.macAddress());

  if (!ParseMac(ROBOT_MAC_STR, robotMac)) {
    Serial.println("# ROBOT_MAC_STR is not a valid MAC address");
    return;
  }

  if (robotMac[0] == 0xAA && robotMac[1] == 0xBB && robotMac[2] == 0xCC) {
    Serial.println("# ROBOT_MAC_STR is still the placeholder.");
    Serial.println("# Power the robot, read the MAC it prints at 115200,");
    Serial.println("# paste it into ROBOT_MAC_STR, reflash this board.");
    return;
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("# esp_now_init failed");
    return;
  }
  esp_now_register_send_cb(OnSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, robotMac, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("# add_peer failed");
    return;
  }

  peerReady = true;
  Serial.print("# forwarding to ");
  Serial.println(ROBOT_MAC_STR);
  Serial.println("# ready");
}

void loop() {
  if (!peerReady) {
    digitalWrite(STATUS_LED, (millis() / 150) % 2);
    return;
  }

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
        esp_now_send(robotMac, buf, sizeof(buf));
        rxState = 0;
        fill = 0;
        lastPacket = millis();
      }
    }
  }

  digitalWrite(STATUS_LED, (millis() - lastPacket) < 400 ? HIGH : LOW);

  if (millis() - lastReport > 2000) {
    lastReport = millis();
    Serial.printf("# sent %lu  failed %lu\n", sent, failed);
  }
}
