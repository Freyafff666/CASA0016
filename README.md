# CASA0016 Smart Study Monitor - "Mom Ball"
## Motivation
During intensive study sessions or deadline periods, students often remain seated for prolonged hours without breaks. This sedentary behavior not only impacts physical health but also leads to deteriorating environmental conditions in the study space. As CO₂ levels rise and lighting becomes inadequate, cognitive performance decreases significantly.

The "Mom Ball" addresses this issue by providing gentle, non-intrusive reminders to maintain a healthy study environment, acting as a digital companion that cares for your well-being during long study sessions.

## Aim
This project creates an interactive environmental guardian for study spaces. By continuously monitoring CO₂ concentration, ambient light, and room occupancy, it provides intuitive feedback through a programmable 16-LED display module (CJMCU-2812-16) and comprehensive data logging via serial interface. When air quality declines or lighting becomes inadequate, the system responds with graduated visual warnings—guiding users toward healthier study habits through gentle reminders to ventilate and illuminate their workspace appropriately.

## Boards and Sensors

1. Arduino MKR WiFi 1010: [[ref](https://store.arduino.cc/products/arduino-uno-wifi-rev2)](https://store.arduino.cc/products/arduino-mkr-wifi-1010)

2. 
<picture>>>>>>>>>>>>>>>

3.

## Construction of the Physical Prototype

## Hardware Components
|Hardware|Description|
|:-------:|:---:|
|Arduino MKR WiFi 1010|Main microcontroller with built-in WiFi capability|
|Adafruit SCD-30|High-precision NDIR CO₂ sensor with temperature and humidity sensing|
|BH1750|Digital light intensity sensor measuring illuminance in lux|
|WPSE314 (PIR Sensor)|Passive infrared motion detector for occupancy sensing|
|CJMCU-2812-16|Integrated 16-LED WS2812B module with built-in microcontroller|
|3.3V/5V Power Supply|External power for stable operation|

## Pin Connection Table
|Hardware|Power|Data|Notes|
|:-------:|:---:|:--:|:--:|
|SCD-30 CO₂ Sensor|5V → Breadboard + rail  GND → Breadboard - rail|	SDA → Pin 11  SCL → Pin 12|I2C address: 0x61|
|BH1750 Light Sensor|5V → Breadboard + rail  GND → Breadboard - rail|SDA → Pin 11  SCL → Pin 12|I2C address: 0x23|
|PIR Sensor (WPSE314)|5V → Breadboard + rail  GND → Breadboard - rail|OUT → Pin 1|Sensitivity adjustable via potentiometer|
|CJMCU-2812-16|VCC → 5V rail  GND → GND rail|DIN → Pin 6 (via 330Ω resistor)|CJMCU module has built-in logic level shifting|

## System Operation Logic
## Sensor Thresholds and Responses
|Parameter|Optimal Range|Warning Threshold|Alert Threshold|LED Response|
|:-------:|:---:|:--:|:--:|:--:|
|CO₂ Concentration|< 800 ppm|800-1500 ppm|> 1500 ppm|White → Yellow → Red|
|Light Intensity|> 50 lux|N/A|< 50 lux|Current color blinks|
|Occupancy Status|Present|N/A|Absent > 30s|LEDs turn off|

## State Machine Behavior
### Boot Sequence (Blue LEDs):
System initializes sensors
Connects to WiFi 
Shows startup animation
### Active Monitoring (Color varies by CO₂ level):
White: CO₂ < 800 ppm (Excellent)
Yellow: CO₂ 800-1500 ppm (Consider ventilation)
Red: CO₂ > 1500 ppm (Ventilate immediately)
### Blinking: Any color blinks when light < 50 lux
Sleep Mode (LEDs off):
Activated when no motion detected for 30 seconds
Conserves power when room is unoccupied


## Process 

1.progress picture

2. prototype

3. Video: 

4.website

## Reflection and Challenges
1. 3D printing -- more stable 


2. AQI

Air quality index includes not only Particulate Matter but also other gases such as SO2, O3, CO and so on. 
It is also possible to find other sensors for more versatile air quality monitoring.



## References:
BH1750：[[ref](https://www.mouser.com/datasheet/2/348/bh1750fvi-e-186247.pdf)]
SCD-30: https://learn.adafruit.com/adafruit-scd30
