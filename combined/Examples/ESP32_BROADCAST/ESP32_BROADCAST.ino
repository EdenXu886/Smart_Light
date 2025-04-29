#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>

#define STATUS_LED 15
#define STATUS_BUTTON 5
#define MAC_SIZE 18

const uint32_t MAGIC_TOKEN = 0xA1B2C3D4;
Preferences prefs;

typedef struct local {
  bool button_down;
  bool light_on;
} local;

typedef struct msg {
  uint32_t token;
  char data[250];
} msg;

class storage {
public:
  local local;
  msg msg;

  storage() {
    local.button_down = false;
    local.light_on = false;
    msg.token = MAGIC_TOKEN;
    msg.data[0] = 0;
  }
};

void format_mac(const uint8_t *mac, char *buf) {
  snprintf(buf, MAC_SIZE, "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

bool is_known_peer(const uint8_t mac[6]) {
  int count = prefs.getUInt("count", 0);
  uint8_t buf[6];
  for (int i = 0; i < count; i++) {
    char key[10];
    snprintf(key, sizeof(key), "peer%d", i);
    prefs.getBytes(key, buf, 6);
    if (memcmp(buf, mac, 6) == 0) return true;
  }
  return false;
}

void register_peer(const uint8_t mac[6]) {
  int count = prefs.getUInt("count", 0);
  char key[10];
  snprintf(key, sizeof(key), "peer%d", count);
  prefs.putBytes(key, mac, 6);
  prefs.putUInt("count", count + 1);

  char buf[MAC_SIZE];
  format_mac(mac, buf);
  Serial.printf("Registered #%d: %s\n", count, buf);
}

class network {
private:
  static storage local_storage;

  static void check_esp_status(esp_err_t result) {
    switch (result) {
      case ESP_OK:
        Serial.println("Broadcast message success");
        break;
      case ESP_ERR_ESPNOW_NOT_INIT:
        Serial.println("ESP-NOW not Init.");
        break;
      case ESP_ERR_ESPNOW_ARG:
        Serial.println("Invalid Argument");
        break;
      case ESP_ERR_ESPNOW_INTERNAL:
        Serial.println("Internal Error");
        break;
      case ESP_ERR_ESPNOW_NO_MEM:
        Serial.println("ESP_ERR_ESPNOW_NO_MEM");
        break;
      case ESP_ERR_ESPNOW_NOT_FOUND:
        Serial.println("Peer not found.");
        break;
      default:
        Serial.println("Unknown error");
        break;
    }
  }

  static void recv_cb(const esp_now_recv_info *info, const uint8_t *data, int len) {
    if (len < (int)sizeof(uint32_t)) return;

    auto *pkt = (msg *)data;
    if (pkt->token != MAGIC_TOKEN) return;  // drop strangers

    if (!is_known_peer(info->src_addr)) {
      register_peer(info->src_addr);
    }

    memcpy(local_storage.msg.data, pkt->data, sizeof(pkt->data));
    local_storage.msg.data[sizeof(pkt->data) - 1] = 0;

    if (strcmp(local_storage.msg.data, "on") == 0) {
      local_storage.local.light_on = true;
    } else if (strcmp(local_storage.msg.data, "off") == 0) {
      local_storage.local.light_on = false;
    }

    digitalWrite(STATUS_LED, local_storage.local.light_on);

    char mac[18];
    format_mac(info->src_addr, mac);
    Serial.printf("From %s: %s\n", mac, local_storage.msg.data);
  }

  static void send_cb(const uint8_t *mac, esp_now_send_status_t status) {
    char buf[18];
    format_mac(mac, buf);
    Serial.printf("Sent to %s: %s\n", buf,
                  status == ESP_NOW_SEND_SUCCESS ? "OK" : "Fail");
  }

public:
  network() {
  }

  void init() {
    WiFi.mode(WIFI_STA);
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());
    WiFi.disconnect();

    if (esp_now_init() == ESP_OK) {
      Serial.println("ESP-NOW Init Success");
      esp_now_register_recv_cb(recv_cb);
      esp_now_register_send_cb(send_cb);
    } else {
      Serial.println("ESP-NOW Init Failed");
      delay(3000);
      ESP.restart();
    }
  }

  void handle_button() {
    if (digitalRead(STATUS_BUTTON)) {
      if (!local_storage.local.button_down) {
        local_storage.local.button_down = true;

        local_storage.local.light_on = !local_storage.local.light_on;
        digitalWrite(STATUS_LED, local_storage.local.light_on);

        local_storage.msg.token = MAGIC_TOKEN;
        strcpy(local_storage.msg.data, local_storage.local.light_on ? "on" : "off");

        broadcast((uint8_t *)&local_storage.msg, sizeof(local_storage.msg));
      }
      delay(300);
    } else {
      local_storage.local.button_down = false;
    }
  }

  void broadcast(const uint8_t *buf, size_t len) {
    uint8_t addr[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, addr, 6);
    if (!esp_now_is_peer_exist(addr)) {
      esp_now_add_peer(&peer);
    }
    check_esp_status(esp_now_send(addr, buf, len));
  }

  storage &get_storage() {
    return local_storage;
  }
};

network network_instance;
storage network::local_storage;

void setup() {
  Serial.begin(115200);
  delay(1000);

  network_instance.init();
  prefs.begin("peers", false);

  pinMode(STATUS_BUTTON, INPUT_PULLUP);
  pinMode(STATUS_LED, OUTPUT);
}

void loop() {
  network_instance.handle_button();
}
