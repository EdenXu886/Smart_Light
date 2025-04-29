#include <WiFi.h>
#include <WebServer.h> // ESP32 Web Server Library
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <BH1750.h>

// WiFi Credentials
const char* ssid = "YourWiFiName";
const char* password = "YourWiFiPassword";

// Web server runs on port 80
WebServer server(80);

// Light sensor
BH1750 lightMeter;

// LED and sensor setup
const int ledPins[] = {18, 19, 21};
const int numLeds = sizeof(ledPins) / sizeof(ledPins[0]);
const int sensorPins[] = {32, 33, 34};

// State tracking
bool ledStates[numLeds] = {false, false, false};
unsigned long ledTimers[numLeds] = {0, 0, 0};
bool isNight = false;
float lux = 0.0;

// Night mode parameters
const float nightModeLuxThreshold = 30.0;
const unsigned long lightOnDuration = 5000; // milliseconds

void setup() {
  Serial.begin(115200);

  Wire.begin();
  lightMeter.begin();

  // Setup LED pins
  for (int i = 0; i < numLeds; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }
  
  // Setup sensor pins
  for (int i = 0; i < numLeds; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected!");
  Serial.println(WiFi.localIP());

  // Define server routes
  server.on("/", handleRoot);
  server.on("/toggle", handleToggleLight);
  server.on("/status", handleStatus);

  server.begin();
}

void loop() {
  server.handleClient(); // Keep server responsive

  // Light and sensor reading
  lux = lightMeter.readLightLevel();
  isNight = (lux < nightModeLuxThreshold);

  if (!isNight) {
    for (int i = 0; i < numLeds; i++) {
      digitalWrite(ledPins[i], LOW);
    }
  } else {
    unsigned long now = millis();
    for (int i = 0; i < numLeds; i++) {
      int sensorValue = digitalRead(sensorPins[i]);
      if (sensorValue == HIGH) {
        ledStates[i] = true;
        ledTimers[i] = now;
      }
      if (ledStates[i]) {
        digitalWrite(ledPins[i], HIGH);
        if (now - ledTimers[i] > lightOnDuration) {
          ledStates[i] = false;
        }
      } else {
        digitalWrite(ledPins[i], LOW);
      }
    }
  }
}

// Web handler: root page (for testing)
void handleRoot() {
  server.send(200, "text/plain", "ESP32 Smart Light Server is running!");
}

// Web handler: toggle light manually
void handleToggleLight() {
  if (server.hasArg("light") && server.hasArg("state")) {
    int lightId = server.arg("light").toInt();
    int state = server.arg("state").toInt();
    if (lightId >= 0 && lightId < numLeds) {
      digitalWrite(ledPins[lightId], state ? HIGH : LOW);
      ledStates[lightId] = (state == 1);
      server.send(200, "text/plain", "Light toggled");
    } else {
      server.send(400, "text/plain", "Invalid light ID");
    }
  } else {
    server.send(400, "text/plain", "Missing parameters");
  }
}

// Web handler: return current status
void handleStatus() {
  String response = "{";
  response += "\"lux\":" + String(lux) + ",";
  response += "\"lights\":[";
  for (int i = 0; i < numLeds; i++) {
    response += (ledStates[i] ? "1" : "0");
    if (i != numLeds - 1) response += ",";
  }
  response += "]}";
  server.send(200, "application/json", response);
}
