# IOT-Enabled-Air-Purification-And-Ventilation-System
This project is an IoT-based smart air quality monitoring and purification system designed to detect harmful gases, monitor temperature and humidity, and automatically control a ventilation fan to maintain healthy indoor air. It integrates multiple sensors, real-time data display, and intelligent fan control using PWM on an ESP32 microcontroller.
🚀 Features
📊 Real-Time Air Quality Monitoring
Uses MQ2, MQ7, MQ135, SDS011, and DHT22 sensors to measure:

Toxic and flammable gases (LPG, CO, smoke, alcohol, ammonia, etc.)

PM2.5 and PM10 particles (SDS011)

Temperature and Humidity (DHT22)

💨 Smart Fan Control

Fan speed is dynamically adjusted using PWM based on pollution level

Helps ventilate the area and purify air efficiently

🖥️ LCD Display Output

Live data shown on 16x2 I2C LCD screen for instant feedback

⚙️ Sensor Fusion Logic

Combines multiple sensor inputs to make accurate decisions on air quality

⚡ Powered by ESP32 with support for further cloud integration (e.g., Blynk, Firebase, etc.)

🔧 Hardware Used
ESP32 WROOM Development Board

MQ2, MQ7, MQ135 Gas Sensors

SDS011 PM2.5/PM10 Sensor

DHT22 Temperature and Humidity Sensor

16x2 I2C LCD Display

12V DC Fan with MOSFET control

Power supply (12V adapter)

💡 How It Works
Sensing: All sensors collect environmental data in real-time.

Evaluation: The ESP32 processes data and calculates AQI-like logic.

Action: Based on thresholds, fan speed is controlled using ledcWrite() (PWM).

Display: Values are shown on LCD for transparency.
