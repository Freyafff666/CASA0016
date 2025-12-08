# CASA0016 Smart Study Monitor - "Mom Ball"
## Motivation
During intensive study sessions or deadline periods, students often remain seated for prolonged hours without breaks. This sedentary behavior not only impacts physical health but also leads to deteriorating environmental conditions in the study space. As CO₂ levels rise and lighting becomes inadequate, cognitive performance decreases significantly.

The "Mom Ball" addresses this issue by providing gentle, non-intrusive reminders to maintain a healthy study environment, acting as a digital companion that cares for your well-being during long study sessions.

## Aim
This project creates an interactive environmental guardian for study spaces. By continuously monitoring CO₂ concentration,temperature, ambient light, and room occupancy, it provides intuitive feedback through a programmable 16-LED display module (CJMCU-2812-16) and comprehensive data logging via serial interface. When air quality declines or lighting becomes inadequate, the system responds with graduated visual warnings—guiding users toward healthier study habits through gentle reminders to ventilate and illuminate their workspace appropriately.


## Hardware Components
|Hardware|Description|
|:-------:|:---:|
|Arduino MKR WiFi 1010|Main microcontroller with built-in WiFi capability|
|Adafruit SCD-30|High-precision NDIR CO₂ sensor with temperature and humidity sensing|
|BH1750|Digital light intensity sensor measuring illuminance in lux|
|WPSE314 (PIR Sensor)|Passive infrared motion detector for occupancy sensing|
|CJMCU-2812-16|Integrated 16-LED WS2812B module with built-in microcontroller|


## Pin Connection Table
|Hardware|Power|Data|Notes|
|:-------:|:---:|:--:|:--:|
|SCD-30 Sensor|5V → Breadboard + rail  GND → Breadboard - rail|	SDA → Pin 11  SCL → Pin 12|I2C address: 0x61|
|BH1750 Light Sensor|5V → Breadboard + rail  GND → Breadboard - rail|SDA → Pin 11  SCL → Pin 12|I2C address: 0x23|
|PIR Sensor (WPSE314)|5V → Breadboard + rail  GND → Breadboard - rail|OUT → Pin 5|Sensitivity adjustable via potentiometer|
|CJMCU-2812-16|VCC → 5V rail  GND → GND rail|DIN → Pin 6 (via 330Ω resistor)|CJMCU module has built-in logic level shifting|

## System Operation Logic
### Sensor Thresholds and Responses
|Parameter|Optimal Range|Warning Threshold|Alert Threshold|LED Response|
|:-------:|:---:|:--:|:--:|:--:|
|CO₂ Concentration|< 800 ppm|800-1500 ppm|> 1500 ppm|White → Yellow → Red|
|Temperature Change|≤18°C|18°C- 28°C|≥28°C|Blue → White → Green|
|Light Intensity|> 400 lux|N/A|< 400 lux|Current color blinks|
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
### Process Monitoring (Color varies by Status)
Blue: Initializing / Standby
White: Operating Normally / In Optimal Range
Green: Process Complete / Optimal Condition Achieved
### Blinking: 
Any color blinks when light < 400 lux
### Sleep Mode (LEDs off):
Activated when no motion detected for 30 seconds
Conserves power when room is unoccupied




### Complete Prototype Assembly and 3D printing
<p align="center">
  
 <img src="https://raw.githubusercontent.com/Freyafff666/CASA0016/main/images/momball1.jpg" width="300"> | <img src="https://raw.githubusercontent.com/Freyafff666/CASA0016/main/images/momball2.jpg" width="300"> 


</p>



### Video



## Reflection and Challenges
1. 3D printing -- more stable 


2. AQI

Air quality index includes not only Particulate Matter but also other gases such as SO2, O3, CO and so on. 
It is also possible to find other sensors for more versatile air quality monitoring.


## References

1.  **Arduino** (n.d.) Arduino MKR WiFi 1010. Available at: https://www.arduino.cc/en/Guide/MKRWiFi1010 (Accessed: 6 December 2025).
2.  **Adafruit** (n.d.) SCD-30 CO₂ / Temperature / Humidity Sensor. Available at: https://www.adafruit.com/product/4867 (Accessed: 6 December 2025).
3.  **Velleman** (2018) VMA314 PIR motion sensor for Arduino: User manual. Available at: https://static.rapidonline.com/pdf/73-4635m_v1.pdf (Accessed: 6 December 2025).
