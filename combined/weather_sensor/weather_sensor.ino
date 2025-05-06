// weather_sensor.ino
/**
 * weather_sensor.ino
 * Functionality:
 * 1. Connect to OpenWeatherMap via Wi-Fi to get weather and sunrise/sunset times.
 * 2. Calculate "turn on lights at sunset" and "turn off lights when clear" logic.
 * 3. Broadcast brightness commands to other nodes using ESP-NOW.
 *
 * Dependencies:
 * - ArduinoJson 6.x
 * - HTTPClient
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_now.h>
#include <time.h>

// —— Wi-Fi & OpenWeatherMap Configuration ——
// Please replace the following information:
const char* ssid      = "YourWiFiName";
const char* password = "YourWiFiPassword";
const char* apiKey    = "YourOpenWeatherMapAPIKey";
// Latitude and longitude (for weather query & sunset calculation)
const char* lat = "YOUR_LATITUDE";
const char* lon = "YOUR_LONGITUDE";

// —— ESP-NOW Broadcast Address ——
// 0xFF for broadcast to all, if you want unicast, enter the target MAC address
uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// Message structure
typedef struct {
  uint8_t brightness;  // 0-255
} msg_t;

// Global variables
time_t sunriseTime = 0;
time_t sunsetTime  = 0;
bool   weatherClear = true;

// Send callback
void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Get weather and parse sunrise/sunset
void getWeather() {
  HTTPClient http;
  String url = String("http://api.openweathermap.org/data/2.5/weather?lat=")
               + lat + "&lon=" + lon + "&appid=" + apiKey;

  http.begin(url);
  int httpCode = http.GET();
  if (httpCode == 200) {
    String payload = http.getString();
    StaticJsonDocument doc;
    auto err = deserializeJson(doc, payload);
    if (!err) {
      // Weather condition
      String mainCond = doc["weather"][0]["main"].as<String>();
      weatherClear = (mainCond == "Clear");
      // Sunrise and sunset Unix timestamps
      sunriseTime = doc["sys"]["sunrise"];
      sunsetTime  = doc["sys"]["sunset"];
      Serial.printf("Weather: %s | Sunrise: %ld | Sunset: %ld\n",
                    weatherClear ? "Clear" : "Not Clear",
                    sunriseTime, sunsetTime);
    } else {
      Serial.println("JSON Parsing Failed");
    }
  } else {
    Serial.printf("HTTP GET Failed, Code=%d\n", httpCode);
  }
  http.end();
}

void setup() {
  Serial.begin(115200);

  // 1. Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

  // 2. Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Error");
    return;
  }
  esp_now_register_send_cb(onSent);
  esp_now_peer_info_t peerInfo = {};
  memcpy(