/*
 * Mom Ball - Smart Study Monitor
 * Hardware: Arduino MKR WiFi 1010 + CJMCU-2812-16 LED Module
 * Sensors: SCD-30 (CO2, Temperature,Humidityxxx), BH1750 (Light), PIR (Motion)
 * 
 * LED Display:
 * - Left 8 LEDs (0-7): CO2 level (White/Yellow/Red)
 * - Right 8 LEDs (8-15): Temperature (Blue/White/Green)
 * - Blinking: When light < 400 lux (all LEDs blink)
 * - Sleep: LEDs off when no motion for 30s
 */

#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SCD30.h>
#include <BH1750.h>

// ====== PIN DEFINITIONS ======
#define PIR_PIN 5           // PIR sensor output (Pin 5)
#define LED_PIN 6           // CJMCU-2812-16 DIN pin
#define NUM_LEDS 16         // CJMCU-2812-16 has 16 LEDs

// ====== THRESHOLDS ======
// CO2 thresholds (ppm)
#define CO2_EXCELLENT 800   // White
#define CO2_WARNING 1500    // Yellow to Red

// Temperature thresholds (°C) 
#define TEMP_LOW 18         // ≤18°C blue
#define TEMP_HIGH 28        // ≥28°C green
#define TEMP_MIN 0          // Minimum temperature for display
#define TEMP_MAX 40         // Maximum temperature for display

// Light thresholds (lux)
#define LIGHT_GOOD 400      // Below this, LEDs blink
#define LIGHT_MIN 0         // Minimum valid light reading
#define LIGHT_MAX 65535     // Maximum valid light reading

// ====== TIMING CONSTANTS ======
#define SENSOR_READ_INTERVAL 2000    // Read sensors every 2s
#define MOTION_TIMEOUT 30000         // 30s timeout for sleep mode
#define PIR_CHECK_INTERVAL 5000      // Check PIR every 5 seconds
#define BLINK_INTERVAL 600           // 600ms for blink effect
#define SERIAL_PRINT_INTERVAL 3000   // Print data every 3s
#define BH1750_MEASURE_INTERVAL 120  // BH1750 measurement interval (ms)

// ====== GLOBAL OBJECTS ======
Adafruit_SCD30 scd30;
Adafruit_NeoPixel leds = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
BH1750 lightMeter;

// ====== STATE VARIABLES ======
enum SystemState { BOOTING, ACTIVE, SLEEP };
SystemState currentState = BOOTING;

enum AirQuality { AIR_EXCELLENT, AIR_GOOD, AIR_POOR, AIR_NO_DATA };
AirQuality airStatus = AIR_NO_DATA;

enum TempColor { TEMP_BLUE, TEMP_WHITE, TEMP_GREEN };
TempColor tempColorState = TEMP_WHITE;

bool motionDetected = false;
bool lightIsGood = true;
bool blinkState = false;
bool scd30Available = false;
bool bh1750Available = false;

unsigned long lastMotionTime = 0;
unsigned long lastSensorRead = 0;
unsigned long lastPIRCheck = 0;      // Last PIR check time
unsigned long lastBlinkTime = 0;
unsigned long lastSerialOutput = 0;
unsigned long bootTime = 0;
unsigned long scd30StartTime = 0;
unsigned long lastLightMeasurement = 0;

float currentCO2 = 0;
float currentTemp = 22.0;          // Default temperature
float currentLight = 0.0;
float lightBuffer[5] = {0};        // Buffer for light value smoothing
uint8_t lightBufferIndex = 0;

// ====== FUNCTION DECLARATIONS ======
void performBootSequence();
void showSuccessAnimation();
void showWarningAnimation();
void showErrorAnimation();
void checkPIRMotion();
void readSensors();
void handleStateTransitions(unsigned long currentTime);
void updateLEDDisplay(unsigned long currentTime);
void updateCO2LEDs(uint32_t color);
void updateTempLEDs(uint32_t color);
void turnOffLEDs();
void printSensorData();
void testPIRSensor();
void scanI2CDevices();
void readSCD30Data();
bool initializeBH1750();
void readBH1750Data();
float smoothLightValue(float newValue);
uint32_t getCO2Color();
uint32_t getTemperatureColor();
void updateTemperatureColor();

// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println();
  Serial.println("========================================");
  Serial.println("       MOM BALL - DUAL DISPLAY MODE");
  Serial.println("========================================");
  Serial.println("Left LEDs: CO2 Level");
  Serial.println("Right LEDs: Temperature (纯色: Blue/White/Green)");
  Serial.println("Temp Rules: ≤18°C=Blue, 18-28°C=White, ≥28°C=Green");
  Serial.println("PIR Detection: Every 5 seconds");
  Serial.println("Auto Sleep: 30 seconds no motion");
  Serial.println("========================================");
  
  bootTime = millis();
  
  // 1. Initialize LED module
  leds.begin();
  leds.setBrightness(100);
  leds.show();
  Serial.println("[LED] CJMCU-2812-16 initialized (16 LEDs)");
  
  // 2. Show boot animation
  performBootSequence();
  
  // 3. Initialize PIR sensor
  pinMode(PIR_PIN, INPUT);
  Serial.println("\n[PIR] Testing motion sensor on Pin 5...");
  Serial.println("  Wave your hand in front of the sensor now!");
  testPIRSensor();
  
  // 4. Initialize I2C and scan for devices
  Wire.begin();
  delay(100);
  scanI2CDevices();
  
  // 5. Initialize BH1750 light sensor
  bh1750Available = initializeBH1750();
  
  // 6. Initialize SCD-30 CO2/Temperature sensor
  Serial.println("\n[SCD-30] Initializing CO2/Temperature sensor...");
  if (scd30.begin()) {
    scd30Available = true;
    scd30StartTime = millis();
    scd30.setMeasurementInterval(2);
    Serial.println("  SCD-30: OK (Address: 0x61)");
    Serial.println("  Functions: CO2, Temperature, Humidity");
    showSuccessAnimation();
  } else {
    scd30Available = false;
    Serial.println("  SCD-30: NOT FOUND");
    Serial.println("  Check: 5V power, SDA(Pin11), SCL(Pin12)");
    showErrorAnimation();
  }
  
  // 7. Initialize light value buffer
  for (int i = 0; i < 5; i++) {
    lightBuffer[i] = 500.0;  // Default value
  }
  
  currentState = ACTIVE;
  lastMotionTime = millis();
  lastLightMeasurement = millis();
  lastPIRCheck = millis();
  
  Serial.println("\n[SYSTEM] Ready for operation!");
  Serial.println("  CO2 Range: <800ppm(White) 800-1500ppm(Yellow) >1500ppm(Red)");
  Serial.println("  Temp Range: ≤18°C=Blue, 18-28°C=White, ≥28°C=Green (纯色)");
  Serial.println("  PIR Detection: Every 5 seconds");
  Serial.println("  Auto Sleep: 30 seconds no motion");
  Serial.println("  Note: SCD-30 may take 30 seconds for first reading");
  Serial.println("========================================\n");
}

// ====== MAIN LOOP ======
void loop() {
  unsigned long currentTime = millis();
  
  // 1. Check PIR motion sensor every 5 seconds
  if (currentTime - lastPIRCheck >= PIR_CHECK_INTERVAL) {
    checkPIRMotion();
    lastPIRCheck = currentTime;
  }
  
  // 2. Handle state transitions
  handleStateTransitions(currentTime);
  
  // 3. State-specific behaviors
  switch (currentState) {
    case BOOTING:
      break;
      
    case ACTIVE:
      // Read sensors periodically
      if (currentTime - lastSensorRead >= SENSOR_READ_INTERVAL) {
        readSensors();
        lastSensorRead = currentTime;
      }
      
      // Update LED display
      updateLEDDisplay(currentTime);
      
      // Print data to serial
      if (currentTime - lastSerialOutput >= SERIAL_PRINT_INTERVAL) {
        printSensorData();
        lastSerialOutput = currentTime;
      }
      break;
      
    case SLEEP:
      // In sleep mode, simply stay asleep
      delay(1000);  // Save CPU resources
      break;
  }
}

// ====== SENSOR FUNCTIONS ======
void testPIRSensor() {
  unsigned long testStart = millis();
  bool detected = false;
  int detectionCount = 0;
  
  Serial.println("  Testing PIR for 10 seconds...");
  Serial.println("  Moving in front of sensor will show 'HIGH'");
  
  while (millis() - testStart < 10000) { // Test for 10 seconds
    int sensorValue = digitalRead(PIR_PIN);
    
    if (sensorValue == HIGH) {
      Serial.println("  HIGH - Motion detected!");
      detected = true;
      detectionCount++;
      delay(1000); // Avoid duplicate counting
    }
    
    delay(100);
  }
  
  if (detected) {
    Serial.print("  PIR: OK - Detected ");
    Serial.print(detectionCount);
    Serial.println(" times");
  } else {
    Serial.println("  PIR: WARNING - No motion detected");
    Serial.println("  Check: Power(5V), GND, Signal(Pin5)");
  }
}

void checkPIRMotion() {
  // Read PIR sensor value
  int sensorValue = digitalRead(PIR_PIN);
  
  // Only record when value changes, to avoid duplicate logs
  static int lastSensorValue = LOW;
  
  if (sensorValue != lastSensorValue) {
    if (sensorValue == HIGH) {
      motionDetected = true;
      lastMotionTime = millis();  // Reset no-motion timer
      
      // Log detection (but don't wake up)
      Serial.println("[PIR] Motion detected (HIGH) - Timer reset");
    } else {
      motionDetected = false;
      Serial.println("[PIR] No motion (LOW)");
    }
    
    lastSensorValue = sensorValue;
  }
}

void scanI2CDevices() {
  Serial.println("\n[I2C] Scanning for devices...");
  byte error, address;
  int found = 0;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("  Found device at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      if (address == 0x61) Serial.print(" <- SCD-30 CO2/Temp Sensor");
      else if (address == 0x23) Serial.print(" <- BH1750 Light Sensor");
      else if (address == 0x5C) Serial.print(" <- BH1750 (Alt Address)");
      
      Serial.println();
      found++;
    }
  }
  
  if (found == 0) {
    Serial.println("  No I2C devices found!");
    Serial.println("  Check connections: SDA(Pin11), SCL(Pin12), Power(5V)");
  } else {
    Serial.print("  Total devices found: ");
    Serial.println(found);
  }
}

bool initializeBH1750() {
  Serial.println("\n[BH1750] Initializing light sensor...");
  
  // Try both possible addresses
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23)) {
    Serial.println("  BH1750: OK (Address: 0x23)");
    return true;
  } else if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x5C)) {
    Serial.println("  BH1750: OK (Address: 0x5C)");
    return true;
  } else {
    Serial.println("  BH1750: NOT FOUND");
    Serial.println("  Check: 5V power, SDA(Pin11), SCL(Pin12)");
    showWarningAnimation();
    return false;
  }
}

void readBH1750Data() {
  unsigned long currentTime = millis();
  
  // Check if measurement is ready (BH1750 needs time between measurements)
  if (currentTime - lastLightMeasurement >= BH1750_MEASURE_INTERVAL) {
    if (lightMeter.measurementReady()) {
      float rawLight = lightMeter.readLightLevel();
      
      // Validate light reading
      if (rawLight >= LIGHT_MIN && rawLight <= LIGHT_MAX) {
        // Apply smoothing
        currentLight = smoothLightValue(rawLight);
        lightIsGood = (currentLight >= LIGHT_GOOD);
      } else {
        // Invalid reading, keep previous value
        Serial.println("[BH1750] Warning: Invalid light reading");
      }
      
      lastLightMeasurement = currentTime;
    }
  }
}

float smoothLightValue(float newValue) {
  // Update circular buffer
  lightBuffer[lightBufferIndex] = newValue;
  lightBufferIndex = (lightBufferIndex + 1) % 5;
  
  // Calculate moving average
  float sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += lightBuffer[i];
  }
  
  return sum / 5.0;
}

void readSensors() {
  // Read light sensor if available
  if (bh1750Available) {
    readBH1750Data();
  } else {
    // Fallback to simulated light value for testing
    static unsigned long lastSimChange = 0;
    if (millis() - lastSimChange > 10000) { // Change every 10 seconds
      currentLight = random(50, 1200); // Random between 50-1200 lux
      lightIsGood = (currentLight >= LIGHT_GOOD);
      lastSimChange = millis();
    }
  }
  
  // Read CO2 and Temperature from SCD-30 if available
  if (scd30Available) {
    readSCD30Data();
  }
}

void readSCD30Data() {
  if (scd30.dataReady()) {
    if (scd30.read()) {
      currentCO2 = scd30.CO2;
      currentTemp = scd30.temperature; // Read temperature from SCD-30
      
      // Update temperature color state
      updateTemperatureColor();
      
      // Determine air quality status based on CO2
      if (currentCO2 < CO2_EXCELLENT) {
        airStatus = AIR_EXCELLENT;
      } else if (currentCO2 <= CO2_WARNING) {
        airStatus = AIR_GOOD;
      } else {
        airStatus = AIR_POOR;
      }
      
      // Successfully read sensor data
      if (currentCO2 > 0) {
        return;
      }
    }
  }
  
  // If we reach here, SCD-30 is still warming up or has no data
  unsigned long currentTime = millis();
  if (currentTime - scd30StartTime < 30000) { // First 30 seconds
    airStatus = AIR_NO_DATA;
  }
}

void updateTemperatureColor() {
  if (currentTemp <= TEMP_LOW) {
    tempColorState = TEMP_BLUE;      // ≤18°C: 蓝色
  } else if (currentTemp < TEMP_HIGH) {
    tempColorState = TEMP_WHITE;     // 18-28°C: 白色
  } else {
    tempColorState = TEMP_GREEN;     // ≥28°C: 绿色
  }
}

// ====== STATE MANAGEMENT ======
void handleStateTransitions(unsigned long currentTime) {
  // ONLY transition from ACTIVE to SLEEP (one-way transition)
  if (currentState == ACTIVE && !motionDetected) {
    // Check if 30 seconds have passed without motion
    if (currentTime - lastMotionTime >= MOTION_TIMEOUT) {
      currentState = SLEEP;
      Serial.println("[SYSTEM] No motion for 30 seconds. Entering sleep mode");
      turnOffLEDs();
    }
  }
}

// ====== LED DISPLAY FUNCTIONS ======
void updateLEDDisplay(unsigned long currentTime) {
  if (currentState != ACTIVE) return;
  
  // Determine colors for both sections
  uint32_t co2Color = getCO2Color();
  uint32_t tempColor = getTemperatureColor();
  
  // Apply blink effect if light is insufficient (affects ALL LEDs)
  if (!lightIsGood) {
    if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
      blinkState = !blinkState;
      lastBlinkTime = currentTime;
    }
    
    if (blinkState) {
      // Display both CO2 and Temperature LEDs
      updateCO2LEDs(co2Color);
      updateTempLEDs(tempColor);
      leds.show();
    } else {
      turnOffLEDs();
    }
  } else {
    // Normal display (no blink)
    updateCO2LEDs(co2Color);
    updateTempLEDs(tempColor);
    leds.show();
  }
}

void updateCO2LEDs(uint32_t color) {
  // Update left 8 LEDs (0-7) for CO2 display
  for (int i = 0; i < 8; i++) {
    leds.setPixelColor(i, color);
  }
}

void updateTempLEDs(uint32_t color) {
  // Update right 8 LEDs (8-15) for temperature display
  for (int i = 8; i < NUM_LEDS; i++) {
    leds.setPixelColor(i, color);
  }
}

void turnOffLEDs() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds.setPixelColor(i, 0);
  }
  leds.show();
}

uint32_t getCO2Color() {
  switch (airStatus) {
    case AIR_EXCELLENT:
      return leds.Color(150, 150, 150); // White
    case AIR_GOOD:
      return leds.Color(255, 200, 0);   // Yellow
    case AIR_POOR:
      return leds.Color(255, 0, 0);     // Red
    case AIR_NO_DATA:
      return leds.Color(0, 0, 100);     // Blue (no data)
    default:
      return leds.Color(50, 50, 50);    // Gray (fallback)
  }
}

uint32_t getTemperatureColor() {
  if (!scd30Available) {
    return leds.Color(100, 100, 100); // Gray if sensor not available
  }
  
  //
  switch (tempColorState) {
    case TEMP_BLUE:
      // ≤18°C: blue
      return leds.Color(0, 0, 255);
      
    case TEMP_WHITE:
      // 18-28°C: white
      return leds.Color(200, 200, 200);
      
    case TEMP_GREEN:
      // ≥28°C: green
      return leds.Color(0, 255, 0);
      
    default:
      return leds.Color(100, 100, 100); // Gray (fallback)
  }
}

// ====== ANIMATION FUNCTIONS ======
void performBootSequence() {
  Serial.println("[BOOT] Starting boot sequence...");
  
  // Left side (CO2) - Blue to White animation
  for (int i = 0; i < 8; i++) {
    leds.setPixelColor(i, leds.Color(0, 0, 100 + i*20));
    leds.show();
    delay(40);
  }
  
  // Right side (Temperature) - 
  // blue
  for (int i = 8; i < 11; i++) {
    leds.setPixelColor(i, leds.Color(0, 0, 200));
    leds.show();
    delay(40);
  }
  // white
  for (int i = 11; i < 14; i++) {
    leds.setPixelColor(i, leds.Color(200, 200, 200));
    leds.show();
    delay(40);
  }
  // green
  for (int i = 14; i < NUM_LEDS; i++) {
    leds.setPixelColor(i, leds.Color(0, 200, 0));
    leds.show();
    delay(40);
  }
  
  delay(300);
  
  // Pulse effect
  for (int b = 0; b <= 100; b += 10) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds.setPixelColor(i, leds.Color(b, b, b));
    }
    leds.show();
    delay(30);
  }
  for (int b = 100; b >= 0; b -= 10) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds.setPixelColor(i, leds.Color(b, b, b));
    }
    leds.show();
    delay(30);
  }
  
  turnOffLEDs();
  delay(200);
}

void showSuccessAnimation() {
  // Green sweep animation
  for (int i = 0; i < NUM_LEDS; i++) {
    leds.setPixelColor(i, leds.Color(0, 150, 0));
    leds.show();
    delay(50);
  }
  delay(200);
  turnOffLEDs();
  delay(200);
}

void showWarningAnimation() {
  // Yellow sweep animation
  for (int i = 0; i < NUM_LEDS; i++) {
    leds.setPixelColor(i, leds.Color(150, 150, 0));
    leds.show();
    delay(50);
  }
  delay(300);
  turnOffLEDs();
  delay(200);
}

void showErrorAnimation() {
  // Fast red blink pattern
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < NUM_LEDS; j++) {
      leds.setPixelColor(j, leds.Color(150, 0, 0));
    }
    leds.show();
    delay(200);
    turnOffLEDs();
    delay(200);
  }
}

// ====== SERIAL OUTPUT ======
void printSensorData() {
  unsigned long uptime = (millis() - bootTime) / 1000;
  
  Serial.print("[");
  Serial.print(uptime);
  Serial.println("s] ENVIRONMENT DATA");
  
  // CO2 Data
  Serial.print("  CO₂: ");
  if (scd30Available && currentCO2 > 0) {
    Serial.print(currentCO2);
    Serial.print(" ppm - ");
    
    switch (airStatus) {
      case AIR_EXCELLENT: Serial.println("✅ Excellent"); break;
      case AIR_GOOD: Serial.println("⚠️ Fair"); break;
      case AIR_POOR: Serial.println("🚨 Poor - Ventilate!"); break;
      case AIR_NO_DATA: Serial.println("⌛ No data"); break;
    }
  } else if (scd30Available) {
    Serial.println("0 ppm (Sensor warming up...)");
  } else {
    Serial.println("Sensor not available");
  }
  
  // Temperature Data
  Serial.print("  Temp: ");
  if (scd30Available && currentTemp > 0) {
    Serial.print(currentTemp);
    Serial.print("°C - ");
    
    switch (tempColorState) {
      case TEMP_BLUE: 
        Serial.print("❄️ Cold (≤18°C) - ");
        Serial.println("🔵 Blue LED");
        break;
      case TEMP_WHITE: 
        Serial.print("✅ Comfortable (18-28°C) - ");
        Serial.println("⚪ White LED");
        break;
      case TEMP_GREEN: 
        Serial.print("🔥 Hot (≥28°C) - ");
        Serial.println("🟢 Green LED");
        break;
    }
  } else if (scd30Available) {
    Serial.println("0°C (Sensor warming up...)");
  } else {
    Serial.println("Sensor not available");
  }
  
  // Light Data
  Serial.print("  Light: ");
  if (bh1750Available) {
    Serial.print(currentLight);
    Serial.print(" lux");
    if (lightIsGood) {
      Serial.println(" ✅ Good");
    } else {
      Serial.println(" ⚠️ Too dark - Turn on lights!");
    }
  } else {
    Serial.print(currentLight);
    Serial.println(" lux (Simulated)");
  }
  
  // Motion and System State
  Serial.print("  Motion: ");
  Serial.println(motionDetected ? "Present" : "Absent");
  
  Serial.print("  Time since last motion: ");
  Serial.print((millis() - lastMotionTime) / 1000);
  Serial.println(" seconds");
  
  Serial.print("  Time to sleep: ");
  if (motionDetected) {
    Serial.println("Timer reset");
  } else {
    int timeLeft = (MOTION_TIMEOUT - (millis() - lastMotionTime)) / 1000;
    if (timeLeft > 0) {
      Serial.print(timeLeft);
      Serial.println(" seconds left");
    } else if (currentState == ACTIVE) {
      Serial.println("Should sleep now");
    } else {
      Serial.println("In sleep mode");
    }
  }
  
  Serial.print("  System: ");
  switch (currentState) {
    case ACTIVE: Serial.println("Active"); break;
    case SLEEP: Serial.println("Sleeping"); break;
    default: Serial.println("Unknown"); break;
  }
  
  Serial.println("----------------------------------------");
}
