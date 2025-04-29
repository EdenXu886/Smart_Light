#include <esp_now.h>
#include <WiFi.h>

typedef struct msg {
  char a[32];
  int b;
  float c;
  bool d;
} msg;

msg test_msg;

void on_data_recv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incoming_data, int len) {
  memcpy(&test_msg, incoming_data, sizeof(test_msg));
  Serial.print("Data received: ");
  Serial.println(len);
  Serial.print("Character Value: ");
  Serial.println(test_msg.a);
  Serial.print("Integer Value: ");
  Serial.println(test_msg.b);
  Serial.print("Float Value: ");
  Serial.println(test_msg.c);
  Serial.print("Boolean Value: ");
  Serial.println(test_msg.d);
  Serial.println();
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("error: failed to initializing ESP_NOW");
    return;
  }

  esp_now_register_recv_cb(on_data_recv);
}

void loop() {
  // put your main code here, to run repeatedly:
}
