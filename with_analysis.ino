#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <VL53L1X.h>  // Standard VL53L1X library
#include <WebServer.h> // Add WebServer library

// Add Blynk related headers and configuration
#define BLYNK_TEMPLATE_ID "TMPL6rwKc30Vn"
#define BLYNK_TEMPLATE_NAME "SmartStreetLight"
#define BLYNK_AUTH_TOKEN "wU3yUKPM7Wb_xVyXavNVMTWSSJ4H6C9X"
#define BLYNK_PRINT Serial  // Add debug output
#include <BlynkSimpleEsp32.h>

// Emergency button and indicator configuration
#define EMERGENCY_BUTTON_PIN 4  // Emergency button GPIO pin - changed from GPIO14 to GPIO4
#define EMERGENCY_LED_PIN 15    // Emergency status indicator pin
#define EMERGENCY_TIMEOUT 3000  // Emergency mode duration (3 seconds), auto switch to dim mode after button press

// WiFi credentials
#define WIFI_SSID "Mi10Ultra"     // Update WiFi network name
#define WIFI_PASSWORD "383838250"  // Update WiFi password

// Blynk and WiFi status variables
bool wifiConnected = false;
unsigned int wifiRetryCount = 0;
#define MAX_WIFI_RETRY 5
#define WIFI_CONNECTION_TIMEOUT 30000  // 30 seconds
#define WIFI_RECONNECT_INTERVAL 20000  // 20 seconds reconnect

// Create Web server object, port 80
WebServer server(80);

// Define LED pin (supports PWM output pins)
#define LED_A 1  // Event indicator light
#define LED_B 2  // Left neighbor connection indicator light
#define LED_C 3  // Right neighbor connection indicator light

// PWM control related parameters
#define PWM_FREQUENCY 5000  // PWM frequency
#define PWM_RESOLUTION 8    // PWM resolution, 8 bits = 0-255
#define PWM_CHANNEL_A 0     // LED_A's PWM channel
#define PWM_CHANNEL_B 1     // LED_B's PWM channel
#define PWM_CHANNEL_C 2     // LED_C's PWM channel

// Streetlight state definition
#define LIGHT_OFF 0   // Off
#define LIGHT_DIM 1   // Dim
#define LIGHT_BRIGHT 2 // Bright
#define LIGHT_EMERGENCY 3 // Emergency mode

// Brightness level (0-255)
#define BRIGHTNESS_OFF 0     // Off
#define BRIGHTNESS_DIM 50    // Dim mode (approximately 20% brightness)
#define BRIGHTNESS_BRIGHT 255 // Bright mode (100% brightness)

// I2C pin definition
#define SDA_PIN 5
#define SCL_PIN 6

// Distance threshold (millimeters)
#define DISTANCE_THRESHOLD 100  // 10cm = 100mm

// VEML6030 sensor address and register definition - moved to front
#define VEML6030_ADDR 0x10      // Default I2C address
#define VEML6030_ADDR_ALT 0x48  // Backup I2C address

// VEML6030 register address
#define VEML6030_REG_ALS_CONF 0x00  // Configuration register
#define VEML6030_REG_ALS 0x04       // Light intensity data register
#define VEML6030_REG_WHITE 0x05     // White light data register
#define VEML6030_REG_ALS_INT_TH_H 0x01  // High threshold register
#define VEML6030_REG_ALS_INT_TH_L 0x02  // Low threshold register

// Light threshold (lux)
#define LUX_THRESHOLD 6.5   // Critical threshold value: lights on below 10.0 lux, lights off above 10.0 lux
#define LUX_HYSTERESIS 5.0   // Hysteresis range, to prevent frequent switching near the threshold

// Sampling settings
#define SAMPLE_COUNT 5          // Sampling times, for averaging
#define SAMPLE_INTERVAL 10      // Sampling interval, in milliseconds

// Motion detection timeout (milliseconds) - changed to 2 seconds
#define MOTION_TIMEOUT 2000  // Time to keep the light on after detecting motion (2 seconds)

// Neighbor motion detection delay (milliseconds)
#define NEIGHBOR_MOTION_DELAY 500  // Neighbor detects motion and delays 0.5 seconds before reacting

// Neighbor's MAC address (needs to be replaced with actual MAC address)
uint8_t leftNeighborMAC[] = {0x30, 0x30, 0xF9, 0x33, 0xF1, 0x90};
uint8_t rightNeighborMAC[] = {0xCC, 0xBA, 0x97, 0x01, 0x29, 0x6C};

// Track neighbor connection status variables
bool leftNeighborConnected = false;
bool rightNeighborConnected = false;

// Track sensor and neighbor events variables
bool distanceEventDetected = false;
bool leftNeighborEvent = false;
bool rightNeighborEvent = false;

// Streetlight control variables
int currentLightState = LIGHT_OFF;
unsigned long lastMotionTime = 0;
unsigned long motionCount = 0;
unsigned long lastNeighborEventTime = 0;
bool pendingNeighborResponse = false;
byte lightSensorAddress = VEML6030_ADDR;  // Default light sensor address

// Emergency mode variables
bool emergencyMode = false;
unsigned long emergencyStartTime = 0;
bool lastButtonState = HIGH; // Assume button uses pull-up resistor (HIGH when not pressed)
unsigned long lastDebounceTime = 0;
#define DEBOUNCE_DELAY 50 // Button debounce delay (milliseconds)
bool brightnessBoost = false; // New: Button-triggered bright mode flag

// Add power consumption calculation constants
#define POWER_OFF 0      // Off state power (W)
#define POWER_DIM 20     // Dim mode power (W)
#define POWER_BRIGHT 100 // Bright mode power (W)

// Send data structure
typedef struct sensor_message {
  uint8_t senderMAC[6];
  bool distanceEvent;
  int lightState;      // Light state information
  float luxValue;      // Light value information
  bool boostSignal;    // Bright mode signal
} sensor_message;

// Create sensor message
sensor_message myData;

// Initialize strong mode flag in structure

// Initialize distance sensor
VL53L1X distanceSensor;

// Configuration values
#define GAIN_2X 0x01     // 2x gain
#define GAIN_4X 0x02     // 4x gain (higher sensitivity)
#define IT_800MS 0x03    // 800ms integration time

// Remove statistical analysis related constants and variables
#define DAY_MS 86400000    // Milliseconds in a day

// Motion detection statistics
struct MotionStats {
  uint16_t dailyCount;     // Motion detection times per day
  uint32_t lastDayReset;   // Last daily count reset time
} motionStats = {0, 0};

// Light level statistics
struct LightStats {
  float dailySum;          // Total light value per day
  uint16_t dailySamples;   // Sampling times per day
  float dailyAverage;      // Average value per day
} lightStats = {0, 0, 0};

// Energy consumption statistics
struct EnergyStats {
  unsigned long offDuration;    // Off state duration
  unsigned long dimDuration;    // Dim state duration
  unsigned long brightDuration; // Bright state duration
  float dailyEnergyUsage;      // Daily energy consumption (Watt-hours)
  unsigned long lastStateChange;// Last state change time
} energyStats = {0, 0, 0, 0, 0};

// Add notification cooldown to prevent spam
#define NOTIFICATION_COOLDOWN 30000  // 30 seconds between notifications
unsigned long lastNotificationTime = 0;

// Add time tracking for emergency events
char lastEmergencyTime[30];

// Define event codes
#define EVENT_EMERGENCY_BUTTON "emergency_button"
#define EVENT_DESCRIPTION "Emergency/Boost Button Pressed!"

// Blink LED A
void blinkLedA() {
  digitalWrite(LED_A, HIGH);
  delay(300);
  digitalWrite(LED_A, LOW);
  delay(300);
}

// Get MAC address function
String getMacAddress() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char macStr[18] = { 0 };
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

// Data send callback function
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Data sent successfully");
  } else {
    Serial.println("Data send failed");
  }
}

// Process events function declaration
void processEvents();

// Neighbor send event function declaration
void sendEventToNeighbors();

// Set light state function - Use standard Arduino method to control brightness
void setLightState(int state) {
  if (state != currentLightState) {
    updateEnergyStats(); // Update energy statistics when state changes
    currentLightState = state;
    
    switch(state) {
      case LIGHT_OFF:
        // All LEDs off
        analogWrite(LED_A, BRIGHTNESS_OFF);
        analogWrite(LED_B, BRIGHTNESS_OFF);
        analogWrite(LED_C, BRIGHTNESS_OFF);
        Serial.println("Light state: OFF");
        break;
        
      case LIGHT_DIM:
        // Dim mode - All LEDs dim (solve only LED_A being on)
        analogWrite(LED_A, BRIGHTNESS_DIM);
        analogWrite(LED_B, BRIGHTNESS_DIM);
        analogWrite(LED_C, BRIGHTNESS_DIM);
        Serial.println("Light state: Dim mode");
        break;
        
      case LIGHT_BRIGHT:
        // Bright mode - All LEDs full on
        analogWrite(LED_A, BRIGHTNESS_BRIGHT);
        analogWrite(LED_B, BRIGHTNESS_BRIGHT);
        analogWrite(LED_C, BRIGHTNESS_BRIGHT);
        Serial.println("Light state: Bright mode (100%)");
        break;
        
      case LIGHT_EMERGENCY:
        // Emergency mode - All LEDs full on, and turn on emergency status indicator
        analogWrite(LED_A, BRIGHTNESS_BRIGHT);
        analogWrite(LED_B, BRIGHTNESS_BRIGHT);
        analogWrite(LED_C, BRIGHTNESS_BRIGHT);
        digitalWrite(EMERGENCY_LED_PIN, HIGH); // Turn on emergency indicator
        Serial.println("Light state: Emergency mode (100%)");
        break;
        
      default:
        Serial.println("Unknown light state");
        break;
    }
  }
}

// Read register value - Optimized version
uint16_t readRegister(uint8_t reg) {
  Wire.beginTransmission(lightSensorAddress);
  Wire.write(reg);
  byte error = Wire.endTransmission(false); // Use false to maintain connection
  
  if (error != 0) {
    Serial.print("Read register 0x");
    Serial.print(reg, HEX);
    Serial.print(" failed, error code: ");
    Serial.println(error);
    return 0;
  }
  
  uint16_t value = 0;
  byte bytesRead = Wire.requestFrom(lightSensorAddress, 2);
  
  if (bytesRead == 2) {
    uint8_t lsb = Wire.read();
    uint8_t msb = Wire.read();
    value = (msb << 8) | lsb;
    
    // Debug output
    if (reg == VEML6030_REG_ALS) { // Only detailed debug for light value
      Serial.print("Read bytes: LSB=0x");
      Serial.print(lsb, HEX);
      Serial.print(", MSB=0x");
      Serial.print(msb, HEX);
      Serial.print(", Combined value=0x");
      Serial.println(value, HEX);
    }
  } else {
    Serial.print("Data request failed, received ");
    Serial.print(bytesRead);
    Serial.println(" bytes");
  }
  
  return value;
}

// Read raw value of light intensity - Optimized version
uint16_t readRawValue() {
  return readRegister(VEML6030_REG_ALS);
}

// Convert raw value to lux
float convertToLux(uint16_t rawValue) {
  if (rawValue == 0) return 0.0;
  
  // Calculate conversion factor based on current gain and integration time
  float luxFactor = 0.0018;  // Conversion factor for 4x gain + 800ms integration time
  float calculatedLux = rawValue * luxFactor;
  
  Serial.print("Raw value: ");
  Serial.print(rawValue);
  Serial.print(", Light intensity: ");
  Serial.print(calculatedLux);
  Serial.println(" lux");
  
  return calculatedLux;
}

// Check if it's dark (below threshold)
bool isDark() {
  float luxValue = convertToLux(readRawValue());
  static bool dark = false;
  
  if (luxValue < (LUX_THRESHOLD - LUX_HYSTERESIS)) {
    dark = true;
  } else if (luxValue > (LUX_THRESHOLD + LUX_HYSTERESIS)) {
    dark = false;
  }
  
  return dark;
}

// Verify if sensor is working properly
bool verifySensor() {
  // Try to read configuration register
  uint16_t config = readRegister(VEML6030_REG_ALS_CONF);
  Serial.print("Current configuration register value: 0x");
  Serial.println(config, HEX);
  
  // Try to read light data
  uint16_t rawLight = readRegister(VEML6030_REG_ALS);
  Serial.print("Current light raw value: ");
  Serial.println(rawLight);
  
  // If any non-zero value can be read, the sensor may be working
  return (config != 0 || rawLight != 0);
}

// Configure VEML6030 light sensor - Optimized version
void configureLightSensor() {
  Serial.println("Configuring light sensor...");
  
  // Build configuration value: Higher gain(4x) and longer integration time(800ms) to improve sensitivity
  uint16_t config = 0;
  config |= (GAIN_4X & 0x03) << 11;    // Gain bit 11-12
  config |= (IT_800MS & 0x0F) << 6;    // Integration time bit 6-9
  config &= ~0x01;                     // Bit0=0, enable sensor
  
  Serial.print("Will write configuration value: 0x");
  Serial.println(config, HEX);
  
  // Write configuration
  Wire.beginTransmission(lightSensorAddress);
  Wire.write(VEML6030_REG_ALS_CONF);
  Wire.write(config & 0xFF);          // Low byte
  Wire.write((config >> 8) & 0xFF);   // High byte
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("Light sensor configuration successful");
    
    // Delay for a while to let configuration take effect
    delay(100);
    
    // Read configuration to confirm if setting is correct
    Wire.beginTransmission(lightSensorAddress);
    Wire.write(VEML6030_REG_ALS_CONF);
    error = Wire.endTransmission(false);
    
    if (Wire.requestFrom(lightSensorAddress, 2) == 2) {
      uint8_t lsb = Wire.read();
      uint8_t msb = Wire.read();
      uint16_t readConfig = (msb << 8) | lsb;
      
      Serial.print("Read back configuration value: 0x");
      Serial.println(readConfig, HEX);
      
      if (readConfig != config) {
        Serial.println("Warning: Read back configuration does not match set!");
      }
    }
  } else {
    Serial.print("Light sensor configuration failed, error code: ");
    Serial.println(error);
  }
  
  // Wait for configuration to take effect
  delay(200);
}

// Update light state based on light level and motion status
void updateLightState() {
  float luxValue = convertToLux(readRawValue());
  bool motionDetected = distanceEventDetected || leftNeighborEvent || rightNeighborEvent;
  unsigned long timeSinceLastMotion = millis() - lastMotionTime;
  
  // Use adjustable timeout time
  if (motionDetected) {
    lastMotionTime = millis();
  }
  
  Serial.print("Light intensity: ");
  Serial.print(luxValue);
  Serial.print(" lux, Motion status: ");
  Serial.print(motionDetected ? "Yes" : "No");
  Serial.print(", Last motion detection time: ");
  Serial.print(timeSinceLastMotion);
  Serial.println("ms");
  
  // Use hysteresis control to prevent frequent switching near critical values
  static bool darkState = false;
  
  // Check if it's dark environment (with hysteresis)
  if (luxValue < (LUX_THRESHOLD - LUX_HYSTERESIS)) {
    darkState = true;
  } else if (luxValue > (LUX_THRESHOLD + LUX_HYSTERESIS)) {
    darkState = false;
  }
  
  // Decide light state based on conditions
  if (!darkState) {
    // Environment light level above threshold, turn off light regardless of motion
    setLightState(LIGHT_OFF);
  }
  else if (darkState) {
    // Environment light level below threshold, decide light mode based on motion status
    if (motionDetected && timeSinceLastMotion < MOTION_TIMEOUT) {
      // Motion detected and within timeout time, set to bright mode
      setLightState(LIGHT_BRIGHT);
    }
    else {
      // No motion or timeout, ensure light in dim mode rather than off
      if (currentLightState != LIGHT_DIM) {
        setLightState(LIGHT_DIM);
      }
    }
  }
}

// New: Send strong mode signal to neighbors
void sendBoostToNeighbors() {
  // Prepare data to send
  uint8_t mac[6];
  WiFi.macAddress(mac);
  memcpy(myData.senderMAC, mac, 6);
  myData.distanceEvent = distanceEventDetected;
  myData.lightState = currentLightState;
  myData.luxValue = convertToLux(readRawValue());
  myData.boostSignal = true; // Set strong mode signal
  
  Serial.println("Sending strong mode signal to neighbors");
  
  // Send to left neighbor
  if (leftNeighborConnected) {
    esp_now_send(leftNeighborMAC, (uint8_t *)&myData, sizeof(myData));
  }
  
  // Send to right neighbor
  if (rightNeighborConnected) {
    esp_now_send(rightNeighborMAC, (uint8_t *)&myData, sizeof(myData));
  }
  
  // Reset signal flag to avoid affecting subsequent normal messages
  myData.boostSignal = false;
}

// Data receive callback function - Modified to handle strong mode signal
void OnDataReceive(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  // Get sender MAC address
  const uint8_t *mac_addr = recv_info->src_addr;
  
  // Convert MAC to string for comparison
  char macStr[18] = { 0 };
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  String receivedMac = String(macStr);
  
  // Build left and right neighbor MAC strings
  char leftMacStr[18] = { 0 };
  sprintf(leftMacStr, "%02X:%02X:%02X:%02X:%02X:%02X", leftNeighborMAC[0], leftNeighborMAC[1], leftNeighborMAC[2], leftNeighborMAC[3], leftNeighborMAC[4], leftNeighborMAC[5]);
  String leftMac = String(leftMacStr);
  
  char rightMacStr[18] = { 0 };
  sprintf(rightMacStr, "%02X:%02X:%02X:%02X:%02X:%02X", rightNeighborMAC[0], rightNeighborMAC[1], rightNeighborMAC[2], rightNeighborMAC[3], rightNeighborMAC[4], rightNeighborMAC[5]);
  String rightMac = String(rightMacStr);
 
  // Check if received data is from left or right neighbor
  sensor_message receivedData;
  bool eventChanged = false;
  
  if (len == sizeof(sensor_message)) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    
    // Check if strong mode signal received
    if (receivedData.boostSignal) {
      Serial.print("Received strong mode signal from ");
      Serial.print(macStr);
      Serial.println("!");
      
      // Trigger local strong mode
      brightnessBoost = true;
      emergencyStartTime = millis();
      setLightState(LIGHT_BRIGHT);
      
      // Update Blynk status
      if (Blynk.connected()) {
        Blynk.virtualWrite(V6, 1); // Temporary strong mode notification
      }
    }
    
    if (receivedMac.equals(leftMac)) {
      leftNeighborConnected = true;
      // Check if event status changed
      eventChanged = (leftNeighborEvent != receivedData.distanceEvent);
      leftNeighborEvent = receivedData.distanceEvent;
      
      if (eventChanged && leftNeighborEvent) {
        Serial.println("Left neighbor detected motion! Light will be turned on after 0.5 seconds delay");
        // Save current time for delayed response in loop
        lastNeighborEventTime = millis();
        pendingNeighborResponse = true;
      }
    }
    else if (receivedMac.equals(rightMac)) {
      rightNeighborConnected = true;
      // Check if event status changed
      eventChanged = (rightNeighborEvent != receivedData.distanceEvent);
      rightNeighborEvent = receivedData.distanceEvent;
      
      if (eventChanged && rightNeighborEvent) {
        Serial.println("Right neighbor detected motion! Light will be turned on after 0.5 seconds delay");
        // Save current time for delayed response in loop
        lastNeighborEventTime = millis();
        pendingNeighborResponse = true;
      }
    }
  } else {
    Serial.println("Received data length does not match, ignored");
  }
  
  // Update connection indicator LEDs
  digitalWrite(LED_B, leftNeighborConnected ? HIGH : LOW);
  digitalWrite(LED_C, rightNeighborConnected ? HIGH : LOW);
  
  // Do not update light state directly here, handle in loop
}

// Check distance sensor
void checkDistanceSensor() {
  if (distanceSensor.dataReady()) {
    int distance = distanceSensor.read(false);
    bool previousEvent = distanceEventDetected;
    
    distanceEventDetected = (distance < DISTANCE_THRESHOLD);
    
    // If new motion event detected
    if (distanceEventDetected && !previousEvent) {
      motionCount++; // Increase motion count
      Serial.print("Motion detected, motionCount: ");
      Serial.println(motionCount);
      motionStats.dailyCount++;
      updateMotionStats();
    }
    
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
    
    // If event status changed, immediately respond and send to neighbors
    if (distanceEventDetected != previousEvent) {
      if (distanceEventDetected) {
        // Detected near object
        Serial.println("Detected distance event! Immediately turn on light");
        motionCount++; // Increase motion count
        lastMotionTime = millis(); // Update last motion time
        
        // Turn on only if in dark environment
        if (isDark()) {
          // Immediately update light state to bright
          setLightState(LIGHT_BRIGHT);
        }
        
        // Immediately send event to neighbors
        sendEventToNeighbors();
      } else {
        // Object left, check if need to switch to dim mode
        Serial.println("Distance event cleared");
        
        // If in dark environment and no other active motion
        if (isDark() && !leftNeighborEvent && !rightNeighborEvent) {
          // Switch to dim mode rather than completely off
          setLightState(LIGHT_DIM);
          Serial.println("No motion, switch to dim mode");
        }
      }
    }
  }
}

// Process events function - Light priority, motion secondary
void processEvents() {
  // If local or neighbor event triggered and light level below threshold
  bool darkEnvironment = isDark();
  bool motionDetected = distanceEventDetected || leftNeighborEvent || rightNeighborEvent;
  unsigned long timeSinceLastMotion = millis() - lastMotionTime;
  
  if (!darkEnvironment) {
    // Environment light level above threshold, turn off light unconditionally, highest priority
    setLightState(LIGHT_OFF);
  }
  else if (darkEnvironment) {
    if (motionDetected && timeSinceLastMotion < MOTION_TIMEOUT) {
      // Motion detected and within timeout time, set to bright mode
      setLightState(LIGHT_BRIGHT);
    }
    else {
      // No motion or timeout, ensure light in dim mode rather than off
      if (currentLightState != LIGHT_DIM) {
        setLightState(LIGHT_DIM);
      }
    }
  }
}

// Neighbor send event function
void sendEventToNeighbors() {
  // Prepare data to send
  uint8_t mac[6];
  WiFi.macAddress(mac);
  memcpy(myData.senderMAC, mac, 6);
  myData.distanceEvent = distanceEventDetected;
  myData.lightState = currentLightState;
  myData.luxValue = convertToLux(readRawValue());
  myData.boostSignal = false; // Ensure strong mode signal false in normal event message
  
  // Send to left neighbor
  if (leftNeighborConnected) {
    esp_now_send(leftNeighborMAC, (uint8_t *)&myData, sizeof(myData));
  }
  
  // Send to right neighbor
  if (rightNeighborConnected) {
    esp_now_send(rightNeighborMAC, (uint8_t *)&myData, sizeof(myData));
  }
  
  Serial.println("Event sent to neighbors");
}

// Scan I2C bus
void scanI2C() {
  byte error, address;
  int deviceCount = 0;
  
  Serial.println("Scanning I2C bus...");
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if(error == 0) {
      Serial.print("Found I2C device, address: 0x");
      if(address < 16) Serial.print("0");
      Serial.println(address, HEX);
      deviceCount++;
    }
  }
  
  if(deviceCount == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" I2C devices");
  }
}

// Wait for at least one neighbor connection
bool waitForNeighbor(unsigned long timeout) {
  unsigned long startTime = millis();
  bool anyNeighborConnected = false;
  
  // Reset connection status
  leftNeighborConnected = false;
  rightNeighborConnected = false;
  
  Serial.println("Waiting for neighbor connection...");
  
  // Send broadcast message to try to discover neighbors
  while (!anyNeighborConnected && (millis() - startTime < timeout)) {
    // Blink LED A to indicate waiting
    blinkLedA();
    
    // Prepare broadcast message
    uint8_t mac[6];
    WiFi.macAddress(mac);
    memcpy(myData.senderMAC, mac, 6);
    myData.distanceEvent = false;
    myData.lightState = currentLightState;
    myData.luxValue = convertToLux(readRawValue());
    
    // Try to send to left neighbor
    esp_now_send(leftNeighborMAC, (uint8_t *)&myData, sizeof(myData));
    delay(200);
    
    // Try to send to right neighbor
    esp_now_send(rightNeighborMAC, (uint8_t *)&myData, sizeof(myData));
    delay(200);
    
    // Check if any neighbor is connected
    anyNeighborConnected = leftNeighborConnected || rightNeighborConnected;
    
    // Update LEDs
    digitalWrite(LED_B, leftNeighborConnected ? HIGH : LOW);
    digitalWrite(LED_C, rightNeighborConnected ? HIGH : LOW);
    
    // Print status every second
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime > 1000) {
      Serial.print("Left neighbor status: ");
      Serial.print(leftNeighborConnected ? "Connected" : "Disconnected");
      Serial.print(", Right neighbor status: ");
      Serial.println(rightNeighborConnected ? "Connected" : "Disconnected");
      lastPrintTime = millis();
    }
  }
  
  // Ensure LED A is off
  digitalWrite(LED_A, LOW);
  
  return anyNeighborConnected;
}

// Connect to WiFi network
void connectWiFi() {
  Serial.println("WiFi: Starting connection...");
  
  // Increase retry count
  wifiRetryCount++;
  
  // Check if maximum retry count reached
  if (wifiRetryCount > MAX_WIFI_RETRY) {
    Serial.println("WiFi: Maximum retry count reached, reset WiFi...");
    WiFi.disconnect(true);
    delay(1000);
    WiFi.mode(WIFI_AP_STA); // Use AP+STA mode to support ESP-NOW and Blynk
    wifiRetryCount = 0;
  }
  
  // Connect to WiFi while keeping ESP-NOW functionality
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  // Set connection timeout
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_CONNECTION_TIMEOUT) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi: Connection successful!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    wifiConnected = true;
    wifiRetryCount = 0;
    
    // Set Web server routes
    setupWebServer();
    
    // Connect Blynk
    connectBlynk();
  } else {
    Serial.println("\nWiFi: Connection timeout, will retry later...");
  }
}

// Set Web server
void setupWebServer() {
  // Set routes
  server.on("/", handleRoot);
  server.on("/on", handleLEDOn);
  server.on("/off", handleLEDOff);
  server.on("/boost", handleBrightBoost); // Change route name from emergency to boost
  server.on("/status", handleStatus);
  
  // Start server
  server.begin();
  Serial.println("HTTP server started");
}

// Handle root path request
void handleRoot() {
  String html = "<!DOCTYPE html>"
               "<html>"
               "<head>"
               "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
               "<title>ESP32 LED Control</title>"
               "<style>"
               "body { font-family: Arial; text-align: center; margin: 0px auto; padding: 20px; }"
               "h1 { color: #0F3376; margin: 30px auto; }"
               ".button { display: inline-block; background-color: #4CAF50; border: none; color: white; padding: 16px 40px; "
               "text-decoration: none; font-size: 24px; margin: 10px; cursor: pointer; border-radius: 5px; }"
               ".button2 { background-color: #D11D53; }"
               ".button3 { background-color: #FF9800; }"
               ".info { margin: 20px; font-size: 18px; }"
               "</style>"
               "</head>"
               "<body>"
               "<h1>ESP32 Streetlight Control</h1>"
               "<p class='info'>Current State: " + getLightStateText() + "</p>"
               "<p><a href='/on'><button class='button'>ON</button></a></p>"
               "<p><a href='/off'><button class='button button2'>OFF</button></a></p>"
               "<p><a href='/boost'><button class='button button3'>BOOST MODE (3s)</button></a></p>"
               "<p class='info'>Device MAC: " + getMacAddress() + "</p>"
               "<p class='info'>IP Address: " + WiFi.localIP().toString() + "</p>"
               "<p class='info'>Light Level: " + String(convertToLux(readRawValue())) + " lux</p>"
               "<p class='info'>WiFi Status: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected") + "</p>"
               "<p class='info'>Blynk Status: " + String(Blynk.connected() ? "Connected" : "Disconnected") + "</p>"
               "<p class='info'>Boost Mode: " + String(brightnessBoost ? "Enabled" : "Disabled") + "</p>"
               "</body>"
               "</html>";
  server.send(200, "text/html", html);
}

// Get current light state text
String getLightStateText() {
  switch(currentLightState) {
    case LIGHT_OFF:
      return "OFF";
    case LIGHT_DIM:
      return "DIM";
    case LIGHT_BRIGHT:
      return "BRIGHT (100%)";
    case LIGHT_EMERGENCY:
      return "EMERGENCY (100%)";
    default:
      return "UNKNOWN";
  }
}

// Handle ON request
void handleLEDOn() {
  setLightState(LIGHT_BRIGHT);
  server.sendHeader("Location", "/");
  server.send(303);
}

// Handle OFF request
void handleLEDOff() {
  setLightState(LIGHT_OFF);
  server.sendHeader("Location", "/");
  server.send(303);
}

// Handle BOOST mode request
void handleBrightBoost() {
  brightnessBoost = true;
  emergencyStartTime = millis();
  setLightState(LIGHT_BRIGHT);
  Serial.println("Web control: Starting 3s BOOST mode");
  
  // Update Blynk status
  if (Blynk.connected()) {
    Blynk.virtualWrite(V6, 1); // BOOST mode notification
  }
  
  server.sendHeader("Location", "/");
  server.send(303);
}

// Handle status request
void handleStatus() {
  String json = "{";
  json += "\"light_state\":" + String(currentLightState) + ",";
  json += "\"light_state_text\":\"" + getLightStateText() + "\",";
  json += "\"light_level\":" + String(convertToLux(readRawValue())) + ",";
  json += "\"is_dark\":" + String(isDark() ? "true" : "false") + ",";
  json += "\"emergency_mode\":" + String(emergencyMode ? "true" : "false") + ",";
  json += "\"mac\":\"" + getMacAddress() + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

// Connect to Blynk server
void connectBlynk() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Blynk: Starting connection...");
    
    // Configure Blynk
    Blynk.config(BLYNK_AUTH_TOKEN, "blynk.cloud", 80);
    
    // Use longer timeout time
    unsigned long blynkTimeout = 10000; // 10 seconds
    bool connected = Blynk.connect(blynkTimeout);
    
    if (connected) {
      Serial.println("Blynk: Connected!");
    } else {
      Serial.println("Blynk: Connection failed, will retry later...");
    }
  } else {
    Serial.println("WiFi not connected, cannot connect Blynk");
  }
}

// Calculate current power consumption
float calculatePowerConsumption() {
  switch(currentLightState) {
    case LIGHT_OFF:
      return POWER_OFF;
    case LIGHT_DIM:
      return POWER_DIM;
    case LIGHT_BRIGHT:
    case LIGHT_EMERGENCY:
      return POWER_BRIGHT;
    default:
      return 0;
  }
}

// Send data to Blynk
void sendSensorDataToBlynk() {
  if (Blynk.connected()) {
    float luxValue = convertToLux(readRawValue());
    float currentPower = calculatePowerConsumption();
    
    // Send to Blynk dashboard
    Blynk.virtualWrite(V0, luxValue);       // Light level
    Blynk.virtualWrite(V1, distanceEventDetected); // Motion detection status
    Blynk.virtualWrite(V2, motionCount);           // Motion counter
    Blynk.virtualWrite(V3, currentLightState);     // Light state value
    Blynk.virtualWrite(V6, emergencyMode ? 1 : 0); // Emergency mode status
    
    // Add light state text display
    String stateText;
    switch(currentLightState) {
      case LIGHT_OFF:
        stateText = "OFF";
        break;
      case LIGHT_DIM:
        stateText = "DIM";
        break;
      case LIGHT_BRIGHT:
        stateText = "BRIGHT (100%)";
        break;
      case LIGHT_EMERGENCY:
        stateText = "EMERGENCY (100%)";
        break;
      default:
        stateText = "UNKNOWN";
    }
    Blynk.virtualWrite(V7, stateText);  // Send light state text
    
    // Send gauge data
    Blynk.virtualWrite(V8, luxValue);     // Light intensity gauge
    Blynk.virtualWrite(V9, currentPower); // Power consumption gauge
    
    // Debug output
    Serial.print("Light Level: ");
    Serial.print(luxValue);
    Serial.print(" lux, Power Usage: ");
    Serial.print(currentPower);
    Serial.println(" W");
  }
}

// Blynk connection event
BLYNK_CONNECTED() {
  Serial.println("Blynk: Connected!");
  // Sync current status to Blynk
  sendSensorDataToBlynk();
}

// Handle emergency button
void checkEmergencyButton() {
  // Read button state
  int reading = digitalRead(EMERGENCY_BUTTON_PIN);
  
  // Output current button state every time, for better debugging
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 1000) { // Output current state every second
    Serial.print("Current button state (GPIO");
    Serial.print(EMERGENCY_BUTTON_PIN);
    Serial.print("): ");
    Serial.println(reading == HIGH ? "Not pressed (HIGH)" : "Pressed (LOW)");
    lastPrintTime = millis();
  }
  
  // Only show message when button state changes
  if (reading != lastButtonState) {
    Serial.print(">>>>>> Button state change: ");
    Serial.println(reading == HIGH ? "Not pressed (HIGH)" : "Pressed (LOW)");
    lastDebounceTime = millis();
  }
  
  // Check if button state is stable
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // If button state is pressed state (LOW)
    if (reading == LOW) {
      // Regardless of previous state, if current is pressed state, trigger BOOST mode
      if (!brightnessBoost) { // If not already in BOOST mode, activate it
        brightnessBoost = true;
        emergencyStartTime = millis();
        
        Serial.println("==============================");
        Serial.println("! Button pressed, triggering 3s BOOST mode !");
        Serial.println("==============================");
        
        // Set to bright mode
        setLightState(LIGHT_BRIGHT);
        
        // Send BOOST mode signal to neighbors
        sendBoostToNeighbors();
        
        // Send emergency status to Blynk (if connected)
        if (Blynk.connected()) {
          Blynk.virtualWrite(V6, 1); // Temporary BOOST mode notification
          
          // Add dashboard notification with cooldown
          unsigned long currentTime = millis();
          if (currentTime - lastNotificationTime >= NOTIFICATION_COOLDOWN) {
            // Get current time in hours:minutes:seconds format
            time_t now;
            time(&now);
            struct tm* timeinfo;
            timeinfo = localtime(&now);
            sprintf(lastEmergencyTime, "Last Press: %02d:%02d:%02d", 
                    timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
            
            // Update emergency event display on dashboard
            Blynk.virtualWrite(V20, lastEmergencyTime); // Show event time
            Blynk.virtualWrite(V21, 1); // Set event indicator ON
            
            // Trigger dashboard event notification
            Blynk.logEvent(EVENT_EMERGENCY_BUTTON, EVENT_DESCRIPTION);
            
            lastNotificationTime = currentTime;
            Serial.println("Emergency event and notification sent to dashboard");
          }
        }
      } else {
        // If already in BOOST mode, refresh timer
        emergencyStartTime = millis();
        Serial.println("Button still pressed, reset 3s timer");
      }
    }
  }
  
  // Save current button state
  lastButtonState = reading;
}

// Update motion detection statistics
void updateMotionStats() {
  unsigned long currentTime = millis();
  
  // Update daily count
  if (currentTime - motionStats.lastDayReset >= DAY_MS) {
    motionStats.dailyCount = 0;
    motionStats.lastDayReset = currentTime;
  }
}

// Update light level statistics
void updateLightStats() {
  float currentLux = convertToLux(readRawValue());
  unsigned long currentTime = millis();
  
  // Update daily statistics
  lightStats.dailySum += currentLux;
  lightStats.dailySamples++;
  
  if (currentTime - motionStats.lastDayReset >= DAY_MS) {
    lightStats.dailyAverage = lightStats.dailySum / lightStats.dailySamples;
    lightStats.dailySum = 0;
    lightStats.dailySamples = 0;
  }
}

// Update energy consumption statistics
void updateEnergyStats() {
  unsigned long currentTime = millis();
  unsigned long duration = currentTime - energyStats.lastStateChange;
  
  // Update duration based on current state
  switch(currentLightState) {
    case LIGHT_OFF:
      energyStats.offDuration += duration;
      break;
    case LIGHT_DIM:
      energyStats.dimDuration += duration;
      break;
    case LIGHT_BRIGHT:
    case LIGHT_EMERGENCY:
      energyStats.brightDuration += duration;
      break;
  }
  
  // Calculate daily energy consumption (Watt-hours)
  float hoursFactor = 1.0 / 3600000; // Convert milliseconds to hours
  energyStats.dailyEnergyUsage = 
    (POWER_OFF * energyStats.offDuration + 
     POWER_DIM * energyStats.dimDuration + 
     POWER_BRIGHT * energyStats.brightDuration) * hoursFactor;
  
  energyStats.lastStateChange = currentTime;
  
  // Reset duration every day
  if (currentTime - motionStats.lastDayReset >= DAY_MS) {
    energyStats.offDuration = 0;
    energyStats.dimDuration = 0;
    energyStats.brightDuration = 0;
  }
}

// Send statistical data to Blynk
void sendStatsToBlynk() {
  if (Blynk.connected()) {
    // Get current timestamp for charts
    unsigned long timestamp = millis() / 1000; // Convert to seconds
    
    // Send data as chart points with timestamps
    Blynk.virtualWrite(V15, timestamp, motionStats.dailyCount);      // Motion events chart
    Blynk.virtualWrite(V16, timestamp, lightStats.dailyAverage);     // Light levels chart
    Blynk.virtualWrite(V17, timestamp, energyStats.dailyEnergyUsage); // Energy usage chart
  }
}

// Peak usage analysis structure
struct PeakAnalysis {
  uint16_t hourlyMotionCount[24];    // Motion count per hour
  uint8_t peakStartHour;             // Start of peak period
  uint8_t peakEndHour;               // End of peak period
};

// Initialize analysis structure
PeakAnalysis peakAnalysis = {0};

// Update peak usage analysis
void updatePeakAnalysis() {
  if (Blynk.connected()) {
    // Get current hour
    time_t now;
    time(&now);
    struct tm* timeinfo = localtime(&now);
    int currentHour = timeinfo->tm_hour;
    
    // Update hourly motion count
    peakAnalysis.hourlyMotionCount[currentHour]++;
    
    // Find peak period (3 consecutive hours with highest activity)
    uint16_t maxActivity = 0;
    uint8_t peakStart = 0;
    
    for (int i = 0; i < 24; i++) {
      uint16_t threeHourActivity = 0;
      // Sum activity for 3 consecutive hours
      for (int j = 0; j < 3; j++) {
        int hour = (i + j) % 24;  // Handle wrap-around
        threeHourActivity += peakAnalysis.hourlyMotionCount[hour];
      }
      
      if (threeHourActivity > maxActivity) {
        maxActivity = threeHourActivity;
        peakStart = i;
      }
    }
    
    peakAnalysis.peakStartHour = peakStart;
    peakAnalysis.peakEndHour = (peakStart + 2) % 24;  // End is 2 hours after start
    
    // Send to Blynk
    String peakTimeRange = String(peakAnalysis.peakStartHour) + ":" + 
                          String(peakAnalysis.peakEndHour);
    Blynk.virtualWrite(V22, peakTimeRange);  // Send as string "HH:HH"
  }
}

void setup() {
  // 初始化串口监视器
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n##########################################");
  Serial.println("ESP32 Xiao 智能路灯节点 - 按钮测试版本");
  Serial.println("##########################################");
  
  // 将设备设置为Wi-Fi模式，同时支持ESP-NOW和Blynk
  WiFi.mode(WIFI_AP_STA);
  
  // 初始化LED为PWM输出模式
  pinMode(LED_A, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(LED_C, OUTPUT);
  
  // 初始化紧急按钮和指示灯
  pinMode(EMERGENCY_BUTTON_PIN, INPUT_PULLUP); // 使用内置上拉电阻
  pinMode(EMERGENCY_LED_PIN, OUTPUT);
  digitalWrite(EMERGENCY_LED_PIN, LOW); // 初始时关闭紧急指示灯
  
  // 测试LED指示灯 - 闪烁以确认工作正常
  for (int i = 0; i < 3; i++) {
    digitalWrite(EMERGENCY_LED_PIN, HIGH);
    delay(200);
    digitalWrite(EMERGENCY_LED_PIN, LOW);
    delay(200);
  }
  
  // 测试按钮状态
  Serial.println("\n====== 按钮测试初始信息 ======");
  Serial.print("初始按钮状态 (GPIO");
  Serial.print(EMERGENCY_BUTTON_PIN);
  Serial.print("): ");
  int btnState = digitalRead(EMERGENCY_BUTTON_PIN);
  Serial.println(btnState == HIGH ? "未按下(HIGH)" : "已按下(LOW)");
  Serial.println("请尝试按下按钮观察状态变化");
  Serial.println("按钮应该连接到GPIO4和GND之间");
  Serial.println("==================================\n");
  
  // 初始时关闭所有LED
  analogWrite(LED_A, BRIGHTNESS_OFF);
  analogWrite(LED_B, BRIGHTNESS_OFF);
  analogWrite(LED_C, BRIGHTNESS_OFF);
  
  // 初始化I2C通信，使用指定的引脚
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // 降低速率提高稳定性
  
  // 扫描I2C设备
  scanI2C();
  
  // 尝试连接光线传感器
  Wire.beginTransmission(lightSensorAddress);
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.print("光传感器在地址0x");
    Serial.print(lightSensorAddress, HEX);
    Serial.println("连接成功");
  } else {
    Serial.println("默认地址未找到传感器，尝试备用地址...");
    
    // 尝试备用地址
    lightSensorAddress = VEML6030_ADDR_ALT;
    Wire.beginTransmission(lightSensorAddress);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("光传感器在备用地址0x");
      Serial.print(lightSensorAddress, HEX);
      Serial.println("连接成功");
    } else {
      Serial.println("两个地址都未找到光线传感器！");
    }
  }
  
  // 配置光线传感器
  configureLightSensor();
  
  // 验证传感器通信
  if (verifySensor()) {
    Serial.println("光传感器验证成功");
    
    // 测试读取光照强度
    uint16_t rawValue = readRawValue();
    float luxValue = convertToLux(rawValue);
    Serial.print("当前光照强度: ");
    Serial.print(luxValue);
    Serial.println(" lux");
  } else {
    Serial.println("光传感器验证失败，将无法根据环境光自动控制");
  }
  
  // 初始化VL53L1X距离传感器
  Serial.println("初始化VL53L1X距离传感器...");
  if (!distanceSensor.init()) {
    Serial.println("VL53L1X初始化失败！");
  } else {
    Serial.println("VL53L1X初始化成功");
    
    // 配置传感器
    distanceSensor.setDistanceMode(VL53L1X::Long);  // 长距离模式
    distanceSensor.setMeasurementTimingBudget(50000);  // 50毫秒的计时预算
    
    // 启动连续测量
    distanceSensor.startContinuous(100);  // 每100毫秒测量一次
  }
  
  // 打印MAC地址
  Serial.print("本机MAC地址: ");
  Serial.println(WiFi.macAddress());
  
  // 初始化ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW初始化失败");
    return;
  }
  
  // 注册回调函数
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataReceive);
  
  // 注册对等点（邻居）
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  
  // 添加左邻居
  memcpy(peerInfo.peer_addr, leftNeighborMAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("添加左邻居失败");
  }
  
  // 添加右邻居
  memcpy(peerInfo.peer_addr, rightNeighborMAC, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("添加右邻居失败");
  }
  
  Serial.println("ESP-NOW初始化完成");
  
  // 等待至少一个邻居连接，超时时间为60秒
  if (waitForNeighbor(60000)) {
    Serial.println("至少有一个邻居连接成功！");
  } else {
    Serial.println("等待邻居连接超时！将继续运行，但可能没有邻居连接。");
  }
  
  // 连接到WiFi网络和Blynk
  connectWiFi();
  
  Serial.println("=====================");
  Serial.println("设置完成，开始正常运行");
}

void loop() {
  // 检查紧急按钮状态 - 最高优先级
  checkEmergencyButton();
  
  // 检查强光模式定时器 - 第二优先级
  // 如果强光模式启动超过3秒，自动切换到弱光模式
  if (brightnessBoost && (millis() - emergencyStartTime > EMERGENCY_TIMEOUT)) {
    Serial.println("==============================");
    Serial.println("! 3秒强光模式结束，切换到弱光模式 !");
    Serial.println("==============================");
    
    brightnessBoost = false;
    
    // 切换到弱光模式
    setLightState(LIGHT_DIM);
    
    // 更新Blynk状态
    if (Blynk.connected()) {
      Blynk.virtualWrite(V6, 0); // 取消强光模式
    }
  }
  
  // 如果不在强光模式中，则执行正常的光照和运动检测逻辑
  if (!brightnessBoost) {
    // 先检查光照级别
    bool currentDarkness = isDark();
    
    if (!currentDarkness) {
      // 光照高于阈值，强制关灯
      setLightState(LIGHT_OFF);
    } else if (currentDarkness && currentLightState == LIGHT_OFF) {
      // 如果是黑暗状态，但灯是关闭的，至少切换到暗光模式
      setLightState(LIGHT_DIM);
    }
    
    // 检查距离传感器
    checkDistanceSensor();
    
    // 处理来自邻居的延迟响应
    if (pendingNeighborResponse && (millis() - lastNeighborEventTime >= NEIGHBOR_MOTION_DELAY)) {
      pendingNeighborResponse = false;
      
      // 只有在黑暗环境下才亮灯
      if (isDark()) {
        Serial.println("邻居运动事件延迟处理，开启灯光");
        lastMotionTime = millis(); // 更新最后运动时间
        setLightState(LIGHT_BRIGHT);
      }
    }
    
    // 更快响应周期
    static unsigned long lastLightUpdate = 0;
    if (millis() - lastLightUpdate > 100) { // 每100ms更新一次灯光状态
      updateLightState();
      lastLightUpdate = millis();
    }
    
    // 更新连接指示灯LED
    digitalWrite(LED_B, leftNeighborConnected ? HIGH : LOW);
    digitalWrite(LED_C, rightNeighborConnected ? HIGH : LOW);
    
    // 处理事件
    processEvents();
    
    // 定期发送状态更新
    static unsigned long lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 1000) { // 每秒发送一次状态更新
      sendEventToNeighbors();
      lastStatusUpdate = millis();
    }
    
    // 一段时间后重置邻居事件，防止长期触发
    static unsigned long lastResetTime = 0;
    if (millis() - lastResetTime > 5000) {  // 每5秒重置一次
      // 只有在运动超时后才重置事件
      if (millis() - lastMotionTime > MOTION_TIMEOUT) {
        if (leftNeighborEvent || rightNeighborEvent) {
          Serial.println("重置邻居事件状态");
          leftNeighborEvent = false;
          rightNeighborEvent = false;
          
          // 重置后，确保黑暗环境中灯是在暗光模式而不是关闭
          if (isDark() && currentLightState != LIGHT_DIM) {
            setLightState(LIGHT_DIM);
          }
        }
        lastResetTime = millis();
      }
    }
  }
  
  // ==== Blynk和WiFi相关功能 ====
  // 如果WiFi已连接，运行Blynk和Web服务器
  if (WiFi.status() == WL_CONNECTED) {
    // 处理Web服务器客户端请求
    server.handleClient();
    
    // 检查Blynk连接
    if (Blynk.connected()) {
      Blynk.run();
      
      // 每10秒向Blynk发送一次状态更新
      static unsigned long lastBlynkUpdate = 0;
      if (millis() - lastBlynkUpdate > 10000) {
        sendSensorDataToBlynk();
        lastBlynkUpdate = millis();
      }
    } else {
      // 如果Blynk未连接，每30秒尝试重连一次
      static unsigned long lastBlynkReconnect = 0;
      if (millis() - lastBlynkReconnect > 30000) {
        Serial.println("Blynk未连接，尝试重连...");
        connectBlynk();
        lastBlynkReconnect = millis();
      }
    }
  } else {
    // 如果WiFi未连接，每20秒尝试重连一次
    static unsigned long lastWiFiReconnect = 0;
    if (millis() - lastWiFiReconnect > WIFI_RECONNECT_INTERVAL) {
      Serial.println("WiFi未连接，尝试重连...");
      connectWiFi();
      lastWiFiReconnect = millis();
    }
  }
  
  // Keep statistics update section but only for peak analysis
  static unsigned long lastStatsUpdate = 0;
  if (millis() - lastStatsUpdate >= 60000) { // Update every minute
    updatePeakAnalysis();  // Keep peak usage analysis
    sendStatsToBlynk();    // Keep sending stats to Blynk
    lastStatsUpdate = millis();
  }
  
  // Reset emergency event indicator after cooldown
  if (Blynk.connected()) {
    static unsigned long lastNotificationReset = 0;
    if (millis() - lastNotificationReset >= NOTIFICATION_COOLDOWN) {
      Blynk.virtualWrite(V21, 0); // Reset event indicator
      lastNotificationReset = millis();
    }
  }
  
  // 减少循环延迟以提高响应速度
  delay(10); // 减少延迟从100ms到10ms
}

 
 
 
 
 
 
 