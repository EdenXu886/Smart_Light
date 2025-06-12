#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <VL53L1X.h>  // Use standard VL53L1X library
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <EEPROM.h>

// WiFi configuration
#define WIFI_SSID "Mi10Ultra"     // WiFi network name
#define WIFI_PASSWORD "383838250"   // WiFi password
#define WIFI_RECONNECT_INTERVAL 5000  // WiFi reconnect interval (ms)

// Flask server configuration
#define FLASK_SERVER "http://172.21.168.161:5050" // Flask server address
#define FLASK_UPDATE_INTERVAL 1000  // Data update interval (ms)

// Define LED pins (PWM supported pins)
#define LED_A 1  // Event indicator LED
#define LED_B 2  // Left neighbor connection indicator LED
#define LED_C 3  // Right neighbor connection indicator LED
#define EMERGENCY_BUTTON_PIN 4  // Emergency button pin
 
// PWM control parameters
#define PWM_FREQUENCY 5000  // PWM frequency
#define PWM_RESOLUTION 8    // PWM resolution, 8 bits = 0-255
#define PWM_CHANNEL_A 0     // PWM channel for LED_A
#define PWM_CHANNEL_B 1     // PWM channel for LED_B
#define PWM_CHANNEL_C 2     // PWM channel for LED_C
 
// Streetlight state definitions
#define LIGHT_OFF 0   // Off
#define LIGHT_DIM 1   // Dim
#define LIGHT_BRIGHT 2 // Bright
 
// Brightness levels (0-255)
#define BRIGHTNESS_OFF 0     // Off
#define BRIGHTNESS_DIM 50    // Dim mode (about 20% brightness)
#define BRIGHTNESS_BRIGHT 255 // Bright mode (100% brightness)
 
// I2C pin definitions
#define SDA_PIN 5
#define SCL_PIN 6
 
// Distance threshold (mm)
#define DISTANCE_THRESHOLD 100  // 10cm = 100mm
 
// VEML6030 sensor address and register definitions - moved to the front
#define VEML6030_ADDR 0x10      // Default I2C address
#define VEML6030_ADDR_ALT 0x48  // Alternate I2C address
 
// VEML6030 register addresses
#define VEML6030_REG_ALS_CONF 0x00  // Configuration register
#define VEML6030_REG_ALS 0x04       // Light intensity data register
#define VEML6030_REG_WHITE 0x05     // White light data register
#define VEML6030_REG_ALS_INT_TH_H 0x01  // High threshold register
#define VEML6030_REG_ALS_INT_TH_L 0x02  // Low threshold register
 
// Light threshold (lux)
#define LUX_THRESHOLD 6.5   // Critical threshold: turn on below 10.0 lux, turn off above 10.0 lux
#define LUX_HYSTERESIS 5.0   // Hysteresis range to prevent frequent switching near the threshold
 
// Sampling settings
#define SAMPLE_COUNT 5          // Number of samples for averaging
#define SAMPLE_INTERVAL 10      // Sampling interval in ms
 
// Motion detection timeout (ms) - changed to 2 seconds
#define MOTION_TIMEOUT 2000  // Keep the light on for 2 seconds after motion is detected
 
// Neighbor motion detection delay (ms)
#define NEIGHBOR_MOTION_DELAY 500  // Delay 0.5 seconds to respond after neighbor detects motion
 
// Neighbor MAC addresses (replace with actual MAC addresses)
uint8_t leftNeighborMAC[] = {0x30, 0x30, 0xF9, 0x33, 0xF1, 0x90};
uint8_t rightNeighborMAC[] = {0xCC, 0xBA, 0x97, 0x01, 0x29, 0x6C};
 
// Variables to track neighbor connection status
bool leftNeighborConnected = false;
bool rightNeighborConnected = false;
 
// Variables to track sensor and neighbor events
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

// New global variables: today's total and last upload date
unsigned long motionCountToday = 0;
int lastUploadDay = -1;
float powerUsageToday = 0.0; // Today's total power usage (Wh)

// Structure for sending data
typedef struct sensor_message {
  uint8_t senderMAC[6];
  bool distanceEvent;
  int lightState;  // Add light state information
  float luxValue;  // Add light value information
  bool boostSignal;
} sensor_message;
 
// Create sensor message
sensor_message myData;
 
// Initialize distance sensor
VL53L1X distanceSensor;
 
// Configuration values
#define GAIN_2X 0x01     // 2x gain
#define GAIN_4X 0x02     // 4x gain (higher sensitivity)
#define IT_800MS 0x03    // 800ms integration time
 
// Supplement global variables
bool brightnessBoost = false;
unsigned long emergencyStartTime = 0;
unsigned long emergencyCount = 0;  // Add emergency count variable
bool isLocalEmergency = false;  // Add flag to distinguish local and remote emergency
unsigned long lastEmergencyTime = 0;  // Add timestamp to prevent duplicate counting

// WiFi connection status variable
bool wifiConnected = false;
unsigned long lastWifiReconnectAttempt = 0;
unsigned long lastFlaskUpdate = 0;

// ========== Serial print throttling ==========
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 2000; // Print key status every 2 seconds

// Helper functions for EEPROM unsigned long
#define EEPROM_SIZE 8
#define MOTIONCOUNT_ADDR 0
#define MOTIONCOUNT_TODAY_ADDR 4
#define LASTUPLOADDAY_ADDR 8

unsigned long readULongFromEEPROM(int address) {
  unsigned long value = 0;
  for (int i = 0; i < 4; i++) {
    value |= (EEPROM.read(address + i) << (8 * i));
  }
  return value;
}
void writeULongToEEPROM(int address, unsigned long value) {
  for (int i = 0; i < 4; i++) {
    EEPROM.write(address + i, (value >> (8 * i)) & 0xFF);
  }
  EEPROM.commit();
}
int readIntFromEEPROM(int address) {
  int value = 0;
  for (int i = 0; i < 4; i++) {
    value |= (EEPROM.read(address + i) << (8 * i));
  }
  return value;
}
void writeIntToEEPROM(int address, int value) {
  for (int i = 0; i < 4; i++) {
    EEPROM.write(address + i, (value >> (8 * i)) & 0xFF);
  }
  EEPROM.commit();
}

// Place in the global variable area to ensure all functions can access
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
#define DEBOUNCE_DELAY 50

// Function to connect to WiFi
void connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  // Wait for connection, up to 10 seconds
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("MAC address: ");
    Serial.println(WiFi.macAddress());
  } else {
    wifiConnected = false;
    Serial.println("\nWiFi connection failed!");
  }
}
 
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
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.print("Data sent to: ");
    Serial.println(macStr);
    
    // Update connection status
    if (memcmp(mac_addr, leftNeighborMAC, 6) == 0) {
      leftNeighborConnected = true;
    } else if (memcmp(mac_addr, rightNeighborMAC, 6) == 0) {
      rightNeighborConnected = true;
    }
  } else {
    Serial.print("Failed to send to ");
    Serial.print(macStr);
    Serial.println("!");
    
    // Update connection status
    if (memcmp(mac_addr, leftNeighborMAC, 6) == 0) {
      leftNeighborConnected = false;
    } else if (memcmp(mac_addr, rightNeighborMAC, 6) == 0) {
      rightNeighborConnected = false;
    }
  }
}
 
// Function declaration for processing events
void processEvents();
 
// Function declaration for sending events to neighbors
void sendEventToNeighbors();
 
// Update light state function - Use standard Arduino method to control brightness
void setLightState(int state) {
  // If state hasn't changed, don't process
  if (state == currentLightState) {
    return;
  }
  
  currentLightState = state;
  
  switch(state) {
    case LIGHT_OFF:
      // All LEDs off
      analogWrite(LED_A, BRIGHTNESS_OFF);
      analogWrite(LED_B, BRIGHTNESS_OFF);
      analogWrite(LED_C, BRIGHTNESS_OFF);
      Serial.println("Light state: Off");
      break;
      
    case LIGHT_DIM:
      // Dim mode - All LEDs dim
      analogWrite(LED_A, BRIGHTNESS_DIM);
      analogWrite(LED_B, BRIGHTNESS_DIM);
      analogWrite(LED_C, BRIGHTNESS_DIM);
      Serial.println("Light state: Dim mode (20% brightness)");
      break;
      
    case LIGHT_BRIGHT:
      // Bright mode - All LEDs full
      analogWrite(LED_A, BRIGHTNESS_BRIGHT);
      analogWrite(LED_B, BRIGHTNESS_BRIGHT);
      analogWrite(LED_C, BRIGHTNESS_BRIGHT);
      Serial.println("Light state: Bright mode (100% brightness)");
      break;
      
    default:
      Serial.println("Unknown light state");
      break;
  }
}
 
// Read register value - Optimized version
uint16_t readRegister(uint8_t reg) {
  Wire.beginTransmission(lightSensorAddress);
  Wire.write(reg);
  byte error = Wire.endTransmission(false); // Use false to keep connection
  
  if (error != 0) {
    Serial.print("Failed to read register 0x");
    Serial.print(reg, HEX);
    Serial.print(", Error code: ");
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
    if (reg == VEML6030_REG_ALS) { // Only detailed debug for light values
      Serial.print("Read bytes: LSB=0x");
      Serial.print(lsb, HEX);
      Serial.print(", MSB=0x");
      Serial.print(msb, HEX);
      Serial.print(", Combined value=0x");
      Serial.println(value, HEX);
    }
  } else {
    Serial.print("Data read failed, received ");
    Serial.print(bytesRead);
    Serial.println(" bytes");
  }
  
  return value;
}
 
// Read raw value from light sensor - Optimized version
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
  
  // Use hysteresis control
  static bool dark = false;
  
  // Use hysteresis logic to switch state
  if (luxValue < (LUX_THRESHOLD - LUX_HYSTERESIS)) {
    dark = true;
  } else if (luxValue > (LUX_THRESHOLD + LUX_HYSTERESIS)) {
    dark = false;
  }
  
  Serial.print("Current light state: ");
  Serial.print(dark ? "Dark" : "Bright");
  Serial.print(" (");
  Serial.print(luxValue);
  Serial.println(" lux)");
  
  return dark;
}
 
// Verify sensor operation
bool verifySensor() {
  // Try to read configuration register
  uint16_t config = readRegister(VEML6030_REG_ALS_CONF);
  Serial.print("Current configuration register value: 0x");
  Serial.println(config, HEX);
  
  // Try to read light data
  uint16_t rawLight = readRegister(VEML6030_REG_ALS);
  Serial.print("Current raw light value: ");
  Serial.println(rawLight);
  
  // If any non-zero value can be read, the sensor may be working
  return (config != 0 || rawLight != 0);
}
 
// Configure VEML6030 light sensor - Optimized version
void configureLightSensor() {
  Serial.println("Configuring light sensor...");
  
  // Build configuration value: Higher gain(4x) and longer integration time(800ms) to improve sensitivity
  uint16_t config = 0;
  config |= (GAIN_4X & 0x03) << 11;    // Gain bits 11-12
  config |= (IT_800MS & 0x0F) << 6;    // Integration time bits 6-9
  config &= ~0x01;                     // Bit0=0, Enable sensor
  
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
    
    // Delay for configuration to take effect
    delay(100);
    
    // Read configuration to confirm if settings are correct
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
    Serial.print("Light sensor configuration failed, Error code: ");
    Serial.println(error);
  }
  
  // Wait for configuration to take effect
  delay(200);
}
 
// Update light state based on light level and motion status
void updateLightState() {
  float luxValue = convertToLux(readRawValue());
  bool motionDetected = distanceEventDetected || leftNeighborEvent || rightNeighborEvent;
  
  // Check the difference between current time and last motion detection time
  unsigned long timeSinceLastMotion = millis() - lastMotionTime;
  
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
      // Motion detected and within timeout, set to bright mode
      setLightState(LIGHT_BRIGHT);
    }
    else {
      // No motion or timeout, ensure light in dim mode instead of off
      setLightState(LIGHT_DIM);
    }
  }
}
 
// Data receive callback function
void OnDataReceive(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  const uint8_t *mac_addr = recv_info->src_addr;
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  
  Serial.print("Received data from ");
  Serial.print(macStr);
  Serial.println("!");
  
  if (len == sizeof(sensor_message)) {
    sensor_message receivedData;
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    
    // Update connection status
    if (memcmp(mac_addr, leftNeighborMAC, 6) == 0) {
      leftNeighborConnected = true;
      if (receivedData.distanceEvent) {
        Serial.println("Left neighbor detected motion!");
        leftNeighborEvent = true;
        lastNeighborEventTime = millis();
        pendingNeighborResponse = true;
      }
    } else if (memcmp(mac_addr, rightNeighborMAC, 6) == 0) {
      rightNeighborConnected = true;
      if (receivedData.distanceEvent) {
        Serial.println("Right neighbor detected motion!");
        rightNeighborEvent = true;
        lastNeighborEventTime = millis();
        pendingNeighborResponse = true;
      }
    }
    
    // Process emergency signal
    if (receivedData.boostSignal && !isLocalEmergency && (millis() - lastEmergencyTime > 5000)) {  // Only process if not local emergency and time since last emergency is more than 5 seconds
      Serial.println("Received emergency signal!");
      brightnessBoost = true;
      emergencyStartTime = millis();
      lastEmergencyTime = millis();  // Update last emergency time
      setLightState(LIGHT_BRIGHT);
    }
  }
  
  // Update connection indicator LEDs
  digitalWrite(LED_B, leftNeighborConnected ? HIGH : LOW);
  digitalWrite(LED_C, rightNeighborConnected ? HIGH : LOW);
}
 
// Check distance sensor
void checkDistanceSensor() {
  // Check if new data is available
  if (distanceSensor.dataReady()) {
    int distance = distanceSensor.read(false); // Don't clear interrupt
    bool previousEvent = distanceEventDetected;
    // Check distance against threshold
    distanceEventDetected = (distance < DISTANCE_THRESHOLD);
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
    // Only increment motionCount when new motion event is detected
    if (distanceEventDetected && !previousEvent) {
      motionCount++; // Total cumulative
      writeULongToEEPROM(MOTIONCOUNT_ADDR, motionCount);
      motionCountToday++; // Today cumulative
      writeULongToEEPROM(MOTIONCOUNT_TODAY_ADDR, motionCountToday);
      Serial.print("Motion detected, motionCount: ");
      Serial.println(motionCount);
      lastMotionTime = millis(); // Update last motion time
      // Only turn on light in dark environment
      if (isDark()) {
        setLightState(LIGHT_BRIGHT);
      }
      sendEventToNeighbors();
    }
    // Process event status change
    if (distanceEventDetected != previousEvent) {
      if (!distanceEventDetected) {
        Serial.println("Distance event cleared");
        if (isDark() && !leftNeighborEvent && !rightNeighborEvent) {
          setLightState(LIGHT_DIM);
          Serial.println("No motion, switch to dim mode");
        }
      }
    }
  }
}
 
// Process events function - Light priority, motion secondary
void processEvents() {
  // If local or neighbor event triggered and environment light level below threshold
  bool darkEnvironment = isDark();
  bool motionDetected = distanceEventDetected || leftNeighborEvent || rightNeighborEvent;
  unsigned long timeSinceLastMotion = millis() - lastMotionTime;
  
  if (!darkEnvironment) {
    // Environment light level above threshold, unconditional off, highest priority
    setLightState(LIGHT_OFF);
  }
  else if (darkEnvironment) {
    if (motionDetected && timeSinceLastMotion < MOTION_TIMEOUT) {
      // Motion detected and within timeout, set to bright mode
      setLightState(LIGHT_BRIGHT);
    }
    else {
      // No motion or timeout, ensure light in dim mode instead of off
      setLightState(LIGHT_DIM);
    }
  }
}
 
// Send event to neighbors function
void sendEventToNeighbors() {
  // Prepare data to send
  uint8_t mac[6];
  WiFi.macAddress(mac);
  memcpy(myData.senderMAC, mac, 6);
  myData.distanceEvent = distanceEventDetected;
  myData.lightState = currentLightState;
  myData.luxValue = convertToLux(readRawValue());
  myData.boostSignal = brightnessBoost;
  
  // Send to left neighbor
  esp_err_t result = esp_now_send(leftNeighborMAC, (uint8_t *)&myData, sizeof(myData));
  if (result != ESP_OK) {
    Serial.println("Failed to send to left neighbor");
    leftNeighborConnected = false;
  }
  
  // Send to right neighbor
  result = esp_now_send(rightNeighborMAC, (uint8_t *)&myData, sizeof(myData));
  if (result != ESP_OK) {
    Serial.println("Failed to send to right neighbor");
    rightNeighborConnected = false;
  }
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
  
  // Send broadcast message to attempt to discover neighbors
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
    
    // Update LED status
    digitalWrite(LED_B, leftNeighborConnected ? HIGH : LOW);
    digitalWrite(LED_C, rightNeighborConnected ? HIGH : LOW);
    
    // Print status every second
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime > 1000) {
      Serial.print("Left neighbor status: ");
      Serial.print(leftNeighborConnected ? "Connected" : "Not connected");
      Serial.print(", Right neighbor status: ");
      Serial.println(rightNeighborConnected ? "Connected" : "Not connected");
      lastPrintTime = millis();
    }
  }
  
  // Ensure LED A is off
  digitalWrite(LED_A, LOW);
  
  return anyNeighborConnected;
}
 
// Process emergency button
void checkEmergencyButton() {
  // Read button state
  int reading = digitalRead(EMERGENCY_BUTTON_PIN);
  
  // Output current button state every time, for better debugging
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 1000) { // Output current state every second
    Serial.print("Current button state (GPIO");
    Serial.print(EMERGENCY_BUTTON_PIN);
    Serial.print("): ");
    Serial.println(reading == HIGH ? "Not pressed(HIGH)" : "Pressed(LOW)");
    lastPrintTime = millis();
  }
  
  // Process button state change
  if (reading != lastButtonState) {
    Serial.print(">>>>>> Button state change: ");
    Serial.println(reading == HIGH ? "Not pressed(HIGH)" : "Pressed(LOW)");
    lastDebounceTime = millis();
  }
  
  // Check if button state is stable
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // If button state is pressed state (LOW)
    if (reading == LOW) {
      // Regardless of previous state, if current is pressed state, trigger bright mode
      if (!brightnessBoost) { // If not already in bright mode, activate it
        // Prevent duplicate counting within short time
        if (millis() - lastEmergencyTime > 5000) {  // 5 seconds not to duplicate counting
          brightnessBoost = true;
          emergencyStartTime = millis();
          isLocalEmergency = true;  // Mark as local emergency
          emergencyCount++;  // Increase emergency count
          lastEmergencyTime = millis();  // Update last emergency time
          
          Serial.println("==============================");
          Serial.println("! Button pressed, trigger 3 seconds bright mode !");
          Serial.printf("Current emergency count: %lu\n", emergencyCount);
          Serial.println("==============================");
          
          // Set to bright mode
          setLightState(LIGHT_BRIGHT);
          
          // Send bright mode signal to neighbors
          sendBoostToNeighbors();
        } else {
          Serial.println("Too close to last emergency, ignore this count");
        }
      } else {
        // If already in bright mode, refresh timer
        emergencyStartTime = millis();
        Serial.println("Button still pressed, reset 3 seconds timer");
      }
    }
  }
  
  // Save current button state
  lastButtonState = reading;
}

// Send Boost signal to neighbors
void sendBoostToNeighbors() {
  Serial.println("Sending emergency signal to neighbors");
  myData.boostSignal = true;
  
  // Send to left neighbor
  if (leftNeighborConnected) {
    esp_err_t result = esp_now_send(leftNeighborMAC, (uint8_t *)&myData, sizeof(myData));
    if (result != ESP_OK) {
      Serial.println("Failed to send emergency signal to left neighbor");
    } else {
      Serial.println("Emergency signal sent to left neighbor");
    }
  } else {
    Serial.println("Left neighbor not connected, cannot send emergency signal");
  }
  
  // Send to right neighbor
  if (rightNeighborConnected) {
    esp_err_t result = esp_now_send(rightNeighborMAC, (uint8_t *)&myData, sizeof(myData));
    if (result != ESP_OK) {
      Serial.println("Failed to send emergency signal to right neighbor");
    } else {
      Serial.println("Emergency signal sent to right neighbor");
    }
  } else {
    Serial.println("Right neighbor not connected, cannot send emergency signal");
  }
  
  myData.boostSignal = false;
}
 
// Send data to Flask server
void sendDataToFlask() {
  if (!wifiConnected) return;
  Serial.println("[Flask] Uploading data...");
  HTTPClient http;
  String url = String(FLASK_SERVER) + "/update";
  StaticJsonDocument<200> doc;
  doc["mac"] = WiFi.macAddress();
  doc["light_state"] = brightnessBoost ? LIGHT_BRIGHT : currentLightState;
  doc["motion_detected"] = distanceEventDetected;
  doc["motion_count"] = motionCount;
  doc["lux_value"] = convertToLux(readRawValue());
  doc["left_neighbor_connected"] = leftNeighborConnected;
  doc["right_neighbor_connected"] = rightNeighborConnected;
  doc["emergency_mode"] = brightnessBoost ? true : false;
  doc["emergency_event"] = brightnessBoost && (millis() - lastEmergencyTime < 5000) ? true : false;  // Mark as true only within 5 seconds of emergency
  doc["emergency_timestamp"] = brightnessBoost ? lastEmergencyTime : 0;  // Use lastEmergencyTime
  // Calculate power_usage based on light_state
  float power = 0.0;
  if (brightnessBoost || currentLightState == LIGHT_BRIGHT) {
    power = 1.0;
  } else if (currentLightState == LIGHT_DIM) {
    power = 0.5;
  }
  // Cumulative today power usage (assuming 1 second upload interval, converted to Wh)
  powerUsageToday += power / 3600.0;
  doc["power_usage"] = powerUsageToday;
  doc["motion_count_today"] = motionCountToday;
  doc["motion_count_total"] = motionCount;
  String jsonString;
  serializeJson(doc, jsonString);
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(2000); // 2 seconds timeout
  int httpCode = http.POST(jsonString);
  Serial.printf("[Flask] http.POST returned: %d\n", httpCode);
  if (httpCode > 0) {
    if (httpCode == HTTP_CODE_OK) {
      Serial.println("[Flask] Upload successful");
    } else {
      Serial.printf("[Flask] HTTP failed, Error code: %d\n", httpCode);
    }
  } else {
    Serial.printf("[Flask] HTTP failed: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
}

void setup() {
  // Initialize serial monitor
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nESP32 Xiao Smart Streetlight Node");
  Serial.println("=====================");
  
  // Set device as Wi-Fi station mode
  WiFi.mode(WIFI_STA);
  
  // Connect to WiFi
  connectToWiFi();
  
  // Initialize LEDs as PWM output mode
  pinMode(LED_A, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(LED_C, OUTPUT);
  
  // Initialize emergency button, using internal pull-up resistor
  pinMode(EMERGENCY_BUTTON_PIN, INPUT_PULLUP);
  Serial.print("Emergency button pin ");
  Serial.print(EMERGENCY_BUTTON_PIN);
  Serial.println(" Initialized as input pull-up mode");
  
  // Initialize all LEDs off
  analogWrite(LED_A, BRIGHTNESS_OFF);
  analogWrite(LED_B, BRIGHTNESS_OFF);
  analogWrite(LED_C, BRIGHTNESS_OFF);
  
  // Initialize I2C communication, using specified pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // Reduce rate for stability
  
  // Scan I2C devices
  scanI2C();
  
  // Try connecting light sensor
  Wire.beginTransmission(lightSensorAddress);
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.print("Light sensor at address 0x");
    Serial.print(lightSensorAddress, HEX);
    Serial.println(" Connected");
  } else {
    Serial.println("Default address did not find sensor, trying alternate address...");
    
    // Try alternate address
    lightSensorAddress = VEML6030_ADDR_ALT;
    Wire.beginTransmission(lightSensorAddress);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Light sensor at alternate address 0x");
      Serial.print(lightSensorAddress, HEX);
      Serial.println(" Connected");
    } else {
      Serial.println("Two addresses did not find light sensor!");
    }
  }
  
  // Configure light sensor
  configureLightSensor();
  
  // Verify sensor communication
  if (verifySensor()) {
    Serial.println("Light sensor communication successful");
    
    // Test reading light intensity
    uint16_t rawValue = readRawValue();
    float luxValue = convertToLux(rawValue);
    Serial.print("Current light intensity: ");
    Serial.print(luxValue);
    Serial.println(" lux");
  } else {
    Serial.println("Light sensor communication failed, unable to control based on ambient light");
  }
  
  // Initialize VL53L1X distance sensor
  Serial.println("Initializing VL53L1X distance sensor...");
  if (!distanceSensor.init()) {
    Serial.println("VL53L1X initialization failed!");
  } else {
    Serial.println("VL53L1X initialization successful");
    
    // Configure sensor
    distanceSensor.setDistanceMode(VL53L1X::Long);  // Long distance mode
    distanceSensor.setMeasurementTimingBudget(50000);  // 50 milliseconds timing budget
    
    // Start continuous measurement
    distanceSensor.startContinuous(20);   // Measure every 20 milliseconds (faster)
  }
  
  // Print MAC address
  Serial.print("This device MAC address: ");
  Serial.println(WiFi.macAddress());
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }
  
  // Register callback functions
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataReceive);
  
  // Register peer (neighbor)
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  
  // Add left neighbor
  memcpy(peerInfo.peer_addr, leftNeighborMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add left neighbor");
  }
  
  // Add right neighbor
  memcpy(peerInfo.peer_addr, rightNeighborMAC, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add right neighbor");
  }
  
  Serial.println("ESP-NOW initialization completed");
  
  // Wait for neighbor connection
  if (waitForNeighbor(60000)) {
    Serial.println("At least one neighbor connected successfully!");
  } else {
    Serial.println("Waiting for neighbor connection timed out! Will continue running, but may not have neighbors connected.");
  }
  
  Serial.println("=====================");
  Serial.println("Setup completed, starting normal operation");

  EEPROM.begin(16);
  motionCount = readULongFromEEPROM(MOTIONCOUNT_ADDR);
  motionCountToday = readULongFromEEPROM(MOTIONCOUNT_TODAY_ADDR);
  lastUploadDay = readIntFromEEPROM(LASTUPLOADDAY_ADDR);
  emergencyCount = readULongFromEEPROM(12);  // Read emergency count from EEPROM
}
 
void loop() {
  // ====== New: Automatic zeroing at 0:00 each day ======
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  int today = timeinfo->tm_mday;
  if (lastUploadDay != today) {
    motionCountToday = 0;
    writeULongToEEPROM(MOTIONCOUNT_TODAY_ADDR, motionCountToday);
    lastUploadDay = today;
    writeIntToEEPROM(LASTUPLOADDAY_ADDR, lastUploadDay);
    powerUsageToday = 0.0;
  }
  // Check WiFi connection status
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnected = false;
    if (millis() - lastWifiReconnectAttempt > WIFI_RECONNECT_INTERVAL) {
      lastWifiReconnectAttempt = millis();
      Serial.println("WiFi connection lost, attempting reconnect...");
      connectToWiFi();
    }
  } else if (!wifiConnected) {
    wifiConnected = true;
    Serial.println("WiFi reconnected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  // Check emergency button
  checkEmergencyButton();
  // Process emergency mode
  if (brightnessBoost) {
    analogWrite(LED_A, BRIGHTNESS_BRIGHT);
    analogWrite(LED_B, BRIGHTNESS_BRIGHT);
    analogWrite(LED_C, BRIGHTNESS_BRIGHT);
    unsigned long emergencyTime = millis() - emergencyStartTime;
    // Also upload data during emergency mode
    if (wifiConnected && (millis() - lastFlaskUpdate > FLASK_UPDATE_INTERVAL)) {
      sendDataToFlask();
      lastFlaskUpdate = millis();
    }
    if (emergencyTime > 3000) {
      brightnessBoost = false;
      isLocalEmergency = false;  // Reset local emergency flag
      Serial.println("Emergency mode ended, returning to normal state");
      analogWrite(LED_A, BRIGHTNESS_OFF);
      analogWrite(LED_B, BRIGHTNESS_OFF);
      analogWrite(LED_C, BRIGHTNESS_OFF);
      currentLightState = LIGHT_OFF;
      leftNeighborEvent = false;
      rightNeighborEvent = false;
      distanceEventDetected = false;
      delay(100);
    }
    // Skip other logic, but don't return, to ensure data upload
  } else {
    // Check light level
    bool currentDarkness = isDark();
    if (!currentDarkness) {
      setLightState(LIGHT_OFF);
    } else if (currentDarkness) {
      bool motionDetected = distanceEventDetected || leftNeighborEvent || rightNeighborEvent;
      unsigned long timeSinceLastMotion = millis() - lastMotionTime;
      if (motionDetected && timeSinceLastMotion < MOTION_TIMEOUT) {
        setLightState(LIGHT_BRIGHT);
      } else {
        setLightState(LIGHT_DIM);
      }
    }
    checkDistanceSensor();
    if (pendingNeighborResponse && (millis() - lastNeighborEventTime >= NEIGHBOR_MOTION_DELAY)) {
      pendingNeighborResponse = false;
      if (isDark()) {
        Serial.println("Neighbor motion event delayed processing, turn on light");
        lastMotionTime = millis();
        setLightState(LIGHT_BRIGHT);
      }
    }
    // Periodic status update (neighbors)
    static unsigned long lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 10000) {
      sendEventToNeighbors();
      lastStatusUpdate = millis();
    }
    // Periodic upload to Flask
    if (wifiConnected && (millis() - lastFlaskUpdate > FLASK_UPDATE_INTERVAL)) {
      sendDataToFlask();
      lastFlaskUpdate = millis();
    }
  }
  // Print key status every 2 seconds
  if (millis() - lastPrintTime > PRINT_INTERVAL) {
    Serial.printf("[Status] Light:%d, Motion:%d, lux:%.2f, Left Neighbor:%d, Right Neighbor:%d, Emergency:%d, Emergency Count:%lu\n", 
      currentLightState, distanceEventDetected, convertToLux(readRawValue()), 
      leftNeighborConnected, rightNeighborConnected, brightnessBoost, emergencyCount);
    lastPrintTime = millis();
  }
  digitalWrite(LED_B, leftNeighborConnected ? HIGH : LOW);
  digitalWrite(LED_C, rightNeighborConnected ? HIGH : LOW);
  delay(1);
  
  // Periodic save emergency count to EEPROM
  static unsigned long lastEmergencySave = 0;
  if (millis() - lastEmergencySave > 60000) {  // Save every minute
    writeULongToEEPROM(12, emergencyCount);
    lastEmergencySave = millis();
  }
}
 
 
