#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>

#define ADDR_LEN 6
#define LMK_LEN 16

//PIN
#define LED_PIN 15
#define NPN_COLLECTOR_PIN 4

//Brightness calculation
#define LUX_CHANGE_THRESHOLD 20
#define MAX_LUX 1500.0
#define GAMMA_VAL 2.2

typedef struct {
  int brightness;
} msg;

typedef struct {
  int pin_no;
  uint8_t brightness;
} light_val;

typedef struct {
  long lux_val;
  long last_lux_val;
} lux_sensor;

uint8_t mother_mac[] = { 0x88, 0x13, 0xBF, 0x82, 0x03, 0x14 };
uint8_t local_master_key[LMK_LEN] = {
  0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80,
  0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0, 0x00
};

//Data
msg received;

light_val test_light = {
  .pin_no = LED_PIN,
  .brightness = 0,
};

void on_data_recv(const esp_now_recv_info_t *info,
                  const uint8_t *incoming_data,
                  int len) {
  memcpy(&received, incoming_data, sizeof(received));
  test_light.brightness = received.brightness;
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("error: failed to initializing ESP_NOW");
    return;
  }

  esp_now_peer_info_t peer_info = {};
  peer_info.channel = 0;
  peer_info.encrypt = true;
  memcpy(peer_info.lmk, local_master_key, LMK_LEN);

  memcpy(peer_info.peer_addr, mother_mac, ADDR_LEN);

  if (esp_now_add_peer(&peer_info) != ESP_OK) {
    Serial.println("Failed to add mother node peer");
  }

  esp_now_register_recv_cb(on_data_recv);

  ledcAttach(test_light.pin_no, 5000, 8);
  pinMode(NPN_COLLECTOR_PIN, INPUT_PULLUP);
}

void loop() {
  ledcWrite(test_light.pin_no, test_light.brightness);
  int state = digitalRead(NPN_COLLECTOR_PIN);
  if (state == HIGH) {
    Serial.println("LED FAILED!");
  } else {
    Serial.println("LED OK");
  }
  delay(1000);
}
