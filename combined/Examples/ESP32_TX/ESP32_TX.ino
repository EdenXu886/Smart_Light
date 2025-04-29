#include <esp_now.h>
#include <WiFi.h>

int int_val;
float float_val;
bool bool_val;

//Device 1 address
uint8_t broadcast_address[] = { 0x88, 0x13, 0xBF, 0x82, 0x03, 0x14 };

typedef struct msg {
  char a[32];
  int b;
  float c;
  bool d;
} msg;

msg test_msg;

esp_now_peer_info_t peer_info;

void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Message status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "success\n" : "failure\n");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("error: failed to initializing ESP_NOW");
    return;
  }

  esp_now_register_send_cb(on_data_sent);

  memcpy(peer_info.peer_addr, broadcast_address, 6);
  peer_info.channel = 0;
  peer_info.encrypt = false;

  if (esp_now_add_peer(&peer_info) != ESP_OK) {
    Serial.println("error: failed to add peer");
    return;
  }
}

void loop() {
  int_val = random(1, 20);
  float_val = 1.3 * int_val;
  bool_val = !bool_val;

  strcpy(test_msg.a, "Welcome to the network!");
  test_msg.b = int_val;
  test_msg.c = float_val;
  test_msg.d = bool_val;

  esp_err_t result = esp_now_send(broadcast_address, (uint8_t *)&test_msg, sizeof(test_msg));

  if (result == ESP_OK)
    Serial.println("send: success");
  else
    Serial.println("send: failed");

  delay(2000);
}
