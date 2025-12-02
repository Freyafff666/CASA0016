/*
 * Mom Ball - Smart Study Monitor
 * Hardware: Arduino MKR WiFi 1010 + CJMCU-2812-16 LED Module
 * Sensors: SCD-30 (CO2), BH1750 (Light), PIR (Motion)
 * 
 * LED States:
 * - Boot: Blue sequence
 * - Active: White/Yellow/Red based on CO2
 * - Blinking: When light < 400 lux
 * - Sleep: LEDs off when no motion for 30s
 */

#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SCD30.h>
#include <BH1750.h>

// ====== PIN DEFINITIONS ======
#define PIR_PIN 1           // PIR sensor output
#define LED_PIN 6           // CJMCU-2812-16 DIN pin
#define NUM_LEDS 16         // CJMCU-2812-16 has 16 LEDs

// ====== THRESHOLDS ======
#define CO2_EXCELLENT 800   // ppm - White
#define CO2_WARNING 1500    // ppm - Yellow to Red
#define LIGHT_GOOD 400      // lux - Below this, LEDs blink
#define LIGHT_MIN 0         // Minimum valid light reading
#define LIGHT_MAX 65535     // Maximum valid light reading

// ====== TIMING CONSTANTS ======
#define SENSOR_READ_INTERVAL 2000    // Read sensors every 2s
#define MOTION_TIMEOUT 30000         // 30s timeout for sleep mode
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

bool motionDetected = false;
bool lightIsGood = true;
bool blinkState = false;
bool scd30Available = false;
bool bh1750Available = false;

unsigned long lastMotionTime = 0;
unsigned long lastSensorRead = 0;
unsigned long lastBlinkTime = 0;
unsigned long lastSerialOutput = 0;
unsigned long bootTime = 0;
unsigned long scd30StartTime = 0;
unsigned long lastLightMeasurement = 0;

float currentCO2 = 0;
float currentLight = 0.0;
float lightBuffer[5] = {0};  // Buffer for light value smoothing
uint8_t lightBufferIndex = 0;

// ====== FUNCTION DECLARATIONS ======
void performBootSequence();
void showSuccessAnimation();
void showWarningAnimation();
void showErrorAnimation();
void showWakeUpAnimation();
void checkMotion();
void readSensors();
void handleStateTransitions(unsigned long currentTime);
void updateLEDDisplay(unsigned long currentTime);
void setAllLEDs(uint32_t color);
void turnOffLEDs();
void printSensorData();
void testPIRSensor();
void scanI2CDevices();
void readSCD30Data();
bool initializeBH1750();
void readBH1750Data();
float smoothLightValue(float newValue);

// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println();
  Serial.println("========================================");
  Serial.println("       MOM BALL - DIAGNOSTIC MODE");
  Serial.println("========================================");
  
  bootTime = millis();
  
  // 1. Initialize LED module
  leds.begin();
  leds.setBrightness(100);
  leds.show();
  Serial.println("[LED] CJMCU-2812-16 initialized");
  
  // 2. Show boot animation
  performBootSequence();
  
  // 3. Initialize and test PIR sensor
  pinMode(PIR_PIN, INPUT);
  Serial.println("\n[PIR] Testing motion sensor...");
  Serial.println("  Wave your hand in front of the sensor now!");
  testPIRSensor();
  
  // 4. Initialize I2C and scan for devices
  Wire.begin();
  delay(100);
  scanI2CDevices();
  
  // 5. Initialize BH1750 light sensor
  bh1750Available = initializeBH1750();
  
  // 6. Initialize SCD-30 CO2 sensor
  Serial.println("\n[SCD-30] Initializing CO2 sensor...");
  if (scd30.begin()) {
    scd30Available = true;
    scd30StartTime = millis();
    scd30.setMeasurementInterval(2);
    Serial.println("  SCD-30: OK (Address: 0x61)");
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
  
  Serial.println("\n[SYSTEM] Ready for operation!");
  Serial.println("  Note: SCD-30 may take 30 seconds for first reading");
  Serial.println("========================================\n");
}

// ====== MAIN LOOP ======
void loop() {
  unsigned long currentTime = millis();
  
  // 1. Check motion sensor
  checkMotion();
  
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
      delay(1000);
      break;
  }
}

// ====== SENSOR FUNCTIONS ======
void testPIRSensor() {
  unsigned long testStart = millis();
  bool detected = false;
  
  while (millis() - testStart < 5000) { // Test for 5 seconds
    if (digitalRead(PIR_PIN) == HIGH) {
      detected = true;
      break;
    }
    delay(100);
  }
  
  if (detected) {
    Serial.println("  PIR: OK - Motion detected!");
  } else {
    Serial.println("  PIR: WARNING - No motion detected");
    Serial.println("  Check: Power(5V), GND, Signal(Pin1)");
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
      
      if (address == 0x61) Serial.print(" <- SCD-30 CO2 Sensor");
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

void checkMotion() {
  bool newMotion = digitalRead(PIR_PIN);
  
  if (newMotion != motionDetected) {
    motionDetected = newMotion;
    if (motionDetected) {
      lastMotionTime = millis();
      
      if (currentState == SLEEP) {
        currentState = ACTIVE;
        Serial.println("[SYSTEM] Waking up from sleep");
        showWakeUpAnimation();
      }
    }
  }
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
  
  // Read CO2 from SCD-30 if available
  if (scd30Available) {
    readSCD30Data();
  }
}

void readSCD30Data() {
  if (scd30.dataReady()) {
    if (scd30.read()) {
      currentCO2 = scd30.CO2;
      
      // Determine air quality status
      if (currentCO2 < CO2_EXCELLENT) {
        airStatus = AIR_EXCELLENT;
      } else if (currentCO2 <= CO2_WARNING) {
        airStatus = AIR_GOOD;
      } else {
        airStatus = AIR_POOR;
      }
      
      // Successfully read CO2 data
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

// ====== STATE MANAGEMENT ======
void handleStateTransitions(unsigned long currentTime) {
  if (currentState == ACTIVE && !motionDetected) {
    if (currentTime - lastMotionTime >= MOTION_TIMEOUT) {
      currentState = SLEEP;
      Serial.println("[SYSTEM] Entering sleep mode");
      turnOffLEDs();
    }
  }
}

// ====== LED DISPLAY FUNCTIONS ======
void updateLEDDisplay(unsigned long currentTime) {
  if (currentState != ACTIVE) return;
  
  // Determine base color based on air quality
  uint32_t baseColor;
  switch (airStatus) {
    case AIR_EXCELLENT:
      baseColor = leds.Color(150, 150, 150); // White
      break;
    case AIR_GOOD:
      baseColor = leds.Color(255, 200, 0);   // Yellow
      break;
    case AIR_POOR:
      baseColor = leds.Color(255, 0, 0);     // Red
      break;
    case AIR_NO_DATA:
      baseColor = leds.Color(0, 0, 150);     // Blue (no data)
      break;
  }
  
  // Apply blink effect if light is insufficient
  if (!lightIsGood) {
    if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
      blinkState = !blinkState;
      lastBlinkTime = currentTime;
    }
    
    if (blinkState) {
      setAllLEDs(baseColor);
    } else {
      turnOffLEDs();
    }
  } else {
    // Normal display (no blink)
    setAllLEDs(baseColor);
  }
}

void setAllLEDs(uint32_t color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds.setPixelColor(i, color);
  }
  leds.show();
}

void turnOffLEDs() {
  setAllLEDs(leds.Color(0, 0, 0));
}

// ====== ANIMATION FUNCTIONS ======
void performBootSequence() {
  Serial.println("[BOOT] Starting boot sequence...");
  
  // Blue scanning animation
  for (int i = 0; i < NUM_LEDS/2; i++) {
    leds.setPixelColor(NUM_LEDS/2 + i, leds.Color(0, 0, 100));
    leds.setPixelColor(NUM_LEDS/2 - i, leds.Color(0, 0, 100));
    leds.show();
    delay(50);
  }
  
  delay(300);
  
  // Blue pulse effect
  for (int b = 0; b <= 100; b += 10) {
    setAllLEDs(leds.Color(0, 0, b));
    delay(30);
  }
  for (int b = 100; b >= 0; b -= 10) {
    setAllLEDs(leds.Color(0, 0, b));
    delay(30);
  }
  
  turnOffLEDs();
  delay(200);
}

void showSuccessAnimation() {
  // Green pulse animation
  for (int i = 0; i < 2; i++) {
    setAllLEDs(leds.Color(0, 150, 0));
    delay(300);
    turnOffLEDs();
    delay(300);
  }
}

void showWarningAnimation() {
  // Yellow pulse animation
  for (int i = 0; i < 3; i++) {
    setAllLEDs(leds.Color(150, 150, 0));
    delay(300);
    turnOffLEDs();
    delay(300);
  }
}

void showErrorAnimation() {
  // Fast red blink pattern
  for (int i = 0; i < 5; i++) {
    setAllLEDs(leds.Color(150, 0, 0));
    delay(200);
    turnOffLEDs();
    delay(200);
  }
}

void showWakeUpAnimation() {
  // Green expanding animation
  for (int i = 0; i < NUM_LEDS/2; i++) {
    leds.setPixelColor(NUM_LEDS/2 + i, leds.Color(0, 100, 0));
    leds.setPixelColor(NUM_LEDS/2 - i, leds.Color(0, 100, 0));
    leds.show();
    delay(80);
  }
  delay(300);
  turnOffLEDs();
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
  
  Serial.print("  System: ");
  switch (currentState) {
    case ACTIVE: Serial.println("Active"); break;
    case SLEEP: Serial.println("Sleeping"); break;
    default: Serial.println("Unknown"); break;
  }
  
  Serial.println("----------------------------------------");
}
