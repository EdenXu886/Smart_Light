#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include "SparkFun_VEML6030_Ambient_Light_Sensor.h"
#include <Wire.h>


#define ADDR_LEN 6
#define MAX_CHILD 20
#define LMK_LEN 16
#define AL_ADDR 0x10

//PIN
#define SDA 21
#define SCL 22
#define LED_PIN 15

// Possible values: .125(1/8), .25(1/4), 1, 2
// Both .125 and .25 should be used in most cases except darker rooms.
// A gain of 2 should only be used if the sensor will be covered by a dark
// glass.
#define GAIN 0.125
// Possible integration times in milliseconds: 800, 400, 200, 100, 50, 25
// Higher times give higher resolutions and should be used in darker light.
#define INTEGRATION_TIME 100
#define LUX_CHANGE_THRESHOLD 100
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

typedef struct {
  int id;
  uint8_t mac[ADDR_LEN];
  bool connected;
} child;

class network {
private:
  std::vector<child> childs;
  uint8_t local_master_key[LMK_LEN] = {
    0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80,
    0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0, 0x00
  };
public:
  network() {
    child child_one = { 1, { 0x88, 0x13, 0xBF, 0x82, 0x04, 0x34 }, false };
    add_child(child_one);
  }

  bool add_child(child c) {
    if (childs.size() >= MAX_CHILD) {
      Serial.printf("can't add anymore child node, max:%d\n", MAX_CHILD);
      return false;
    }
    childs.push_back(c);
    return true;
  }

  child* get_child_by_id(int id) {
    for (size_t i = 0; i < childs.size(); ++i) {
      if (childs[i].id == id) {
        return &childs[i];
      }
    }
    return nullptr;
  }

  child* get_child_by_index(int index) {
    if (childs.size() <= 0) {
      Serial.println("There are no registered child nodes available");
      return nullptr;
    }
    return &childs[index];
  }

  int get_childs_count() {
    return childs.size();
  }

  uint8_t* get_lmk() {
    return local_master_key;
  }
};

SparkFun_Ambient_Light light(AL_ADDR);
esp_now_peer_info_t peer_info;

//Data
light_val test_light = {
  .pin_no = LED_PIN,
  .brightness = 0,
};

lux_sensor lux = {
  .lux_val = 0,
  .last_lux_val = -1
};

network network;

bool lux_above_threshold(long lux_val) {
  return lux.last_lux_val >= 0 && abs(lux_val - lux.last_lux_val) < LUX_CHANGE_THRESHOLD;
}

uint8_t cal_brightness(long lux_val) {
  lux_val = constrain(lux_val, 0L, (long)MAX_LUX);

  if (lux_above_threshold(lux_val)) {
    lux_val = lux.last_lux_val;
  }

  const long BOOST_LUX = 100;
  if (lux_val <= BOOST_LUX) {
    lux.last_lux_val = lux_val;
    return 255;
  }

  float norm = float(lux_val - BOOST_LUX) / float(MAX_LUX - BOOST_LUX);
  norm = constrain(norm, 0.0, 1.0);

  float log_val = log10(norm * 9.0 + 1.0);

  float gamma_corrected = pow(log_val, 1.0 / GAMMA_VAL);

  int pwm = int(constrain(gamma_corrected * 255.0 + 0.5, 0, 255));
  uint8_t pwm_inverted = 255 - pwm;

  lux.last_lux_val = lux_val;
  return pwm_inverted;
}

void on_data_sent(const uint8_t* mac, esp_now_send_status_t status) {
  Serial.print("Message status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "success" : "failure");
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  memset(&peer_info, 0, sizeof(peer_info));
  peer_info.channel = 0;
  peer_info.encrypt = true;
  memcpy(peer_info.lmk, network.get_lmk(), LMK_LEN);

  for (int i = 0; i < network.get_childs_count(); i++) {
    child* child = network.get_child_by_index(i);
    memcpy(peer_info.peer_addr, child->mac, ADDR_LEN);
    if (esp_now_add_peer(&peer_info) == ESP_OK) {
      Serial.printf("Added child #%d as encrypted peer: %02X:%02X:%02X:%02X:%02X:%02X\n",
                    child->mac,
                    peer_info.peer_addr[0], peer_info.peer_addr[1],
                    peer_info.peer_addr[2], peer_info.peer_addr[3],
                    peer_info.peer_addr[4], peer_info.peer_addr[5]);
      child->connected = true;
    } else {
      Serial.printf("Failed to add peer for child %d\n", network.get_child_by_index(i)->id);
      child->connected = false;
    }
  }

  esp_now_register_send_cb(on_data_sent);

  Wire.begin(SDA, SCL);

  if (light.begin()) {
    Serial.println("LDR connected successfully");
  } else {
    Serial.println("could not communicate with the sensor!");
    return;
  }

  light.setGain(GAIN);
  light.setIntegTime(INTEGRATION_TIME);

  ledcAttach(test_light.pin_no, 5000, 8);
}

void loop() {
  lux.lux_val = light.readLight();
  //Serial.printf("LDR: %0.2f Lux\n", lux.lux_val);

  test_light.brightness = cal_brightness(lux.lux_val);
  //Serial.printf("Brightness:%d\n", test_light.brightness);

  ledcWrite(test_light.pin_no, test_light.brightness);

  msg transmit_msg{ .brightness = test_light.brightness };

  if (lux_above_threshold(lux.lux_val)) {
    for (int i = 0; i < network.get_childs_count(); i++) {
      child* child = network.get_child_by_index(i);

      if (child->connected) {
        esp_err_t result = esp_now_send(child->mac, (uint8_t*)&transmit_msg, sizeof(transmit_msg));
      }
    }
  }

  delay(100);
}
