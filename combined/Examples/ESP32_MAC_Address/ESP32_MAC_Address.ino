#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  delay(1000);
  Serial.print("MacAddress:");
  Serial.println(WiFi.macAddress());
}

void loop() {
  // put your main code here, to run repeatedly:

}
