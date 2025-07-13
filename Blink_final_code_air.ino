#include <GP2Y1010AU0F.h>
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <SDS011.h>
#include <HardwareSerial.h>

#define BLYNK_TEMPLATE_ID "TMPL32iqcF-A0"
#define BLYNK_TEMPLATE_NAME "Air Purification System"
#define BLYNK_AUTH_TOKEN "-ozdUFSQfRoSF8YGwBDgZGmg1n009bSC"

GP2Y1010AU0F dustSensor(15, 35);

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "OnePlus 11R";
char pass[] = "Sri191205";
//char auth[] = "-ozdUFSQfRoSF8YGwBDgZGmg1n009bSC";

int fanMode = 0;         // 0 = Auto, 1 = Manual
int manualFanSpeed = 0;  // 0–100

BlynkTimer timer;

BLYNK_WRITE(V7) {  // Fan Mode switch
  fanMode = param.asInt();
}

BLYNK_WRITE(V8) {  // Manual speed slider
  manualFanSpeed = param.asInt();
}

// LCD setup for I2C 16x2
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 columns, 2 rows

// DHT22 setup
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// MQ pins
#define MQ2_PIN 32
#define MQ7_PIN 33
#define MQ135_PIN 34

// Motor control
#define IN1 25
#define IN2 26
#define ENA 27

// SDS011
SDS011 sds;
HardwareSerial sdsSerial(2); // UART2

// Sensor values
float pm25, pm10;
int screen = 0; // for rotating display screens

void setup() {
  Serial.begin(115200);
  Wire.begin();
  lcd.begin(16, 2);
  lcd.backlight();

  dht.begin();
  sdsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX, TX
  sds.begin(&sdsSerial);
  dustSensor.begin();

  pinMode(MQ2_PIN, INPUT);
  pinMode(MQ7_PIN, INPUT);
  pinMode(MQ135_PIN, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  ledcAttach(27, 1000, 8);  // Attach PWM to ENA

  // WiFi + Blynk setup
  WiFi.begin(ssid, pass);
  Blynk.config(BLYNK_AUTH_TOKEN); // Only sets auth token, does not block
}

void loop() {
  static unsigned long lastReconnectAttempt = 0;
  const unsigned long reconnectInterval = 5000; // check every 5 sec

  // Manage WiFi reconnection
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastReconnectAttempt > reconnectInterval) {
      Serial.println("🔄 Attempting WiFi reconnect...");
      WiFi.begin(ssid, pass);
      lastReconnectAttempt = millis();
    }
  }

  // Manage Blynk reconnection
  if (WiFi.status() == WL_CONNECTED && !Blynk.connected()) {
    if (millis() - lastReconnectAttempt > reconnectInterval) {
      Serial.println("🔄 Attempting Blynk reconnect...");
      Blynk.connect(2000);  // try connecting for 2 seconds
    }
  }

  // Run Blynk only if connected
  if (Blynk.connected()) {
    Blynk.run();
  }

  // === All your sensor reading + LCD + fan control logic below ===
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  float mq2 = analogRead(MQ2_PIN);
  float mq7 = analogRead(MQ7_PIN);
  float mq135 = analogRead(MQ135_PIN);
  float dust = dustSensor.read();

  float norm_mq2 = (mq2 / 4095.0) * 100;
  float norm_mq7 = (mq7 / 4095.0) * 100;
  float norm_mq135 = (mq135 / 4095.0) * 100;
  float norm_temp = temp;
  float norm_hum = hum;
  float temp_scaled = constrain((temp / 50.0) * 100.0, 0, 100);
  float norm_dust = constrain((dust/500.0)*100.0, 0, 100.0);

  float fusion = (0.25 * norm_mq2) + 
                 (0.20 * norm_mq7) + 
                 (0.25 * norm_mq135) + 
                 (0.20 * norm_dust) + 
                 (0.10 * norm_hum);

  int pwm;
  if (fanMode == 1) {
      if (fusion < 0.3) {
          pwm = 0; // Air is clean
      } else if (fusion < 0.5) {
          pwm = 77; // 30% of 255
      } else if (fusion < 0.7) {
          pwm = 153; // 60%
      } else if (fusion < 0.85) {
          pwm = 217; // 85%
      } else {
          pwm = 255; // 100%
      }
  } else {
    pwm = map(manualFanSpeed, 0, 100, 0, 255);
  }

  pwm = constrain(pwm, 0, 255);
  ledcWrite(27, pwm);
  int pwm_percent = map(pwm, 0, 255, 0, 100);

  // Send data to Blynk only if connected
  if (Blynk.connected()) {
    Blynk.virtualWrite(V0, norm_mq135);
    Blynk.virtualWrite(V1, norm_mq7); 
    Blynk.virtualWrite(V2, norm_mq2);
    Blynk.virtualWrite(V3, temp); 
    Blynk.virtualWrite(V4, hum);
    Blynk.virtualWrite(V5, dust); 
    //Blynk.virtualWrite(V6, pm10);
    Blynk.virtualWrite(V9, pwm_percent);
  }

  // LCD Display - switch screen every 2 seconds
  lcd.clear();
  switch (screen) {
    case 0:
      lcd.setCursor(0, 0); lcd.print("Temp(C):"); lcd.print(temp, 1); lcd.print("C");
      lcd.setCursor(0, 1); lcd.print("Hum(%):"); lcd.print(hum, 1); lcd.print("%");
      break;
    case 1:
      lcd.setCursor(0, 0); lcd.print("PM2.5:"); lcd.print(dust, 1);
      lcd.setCursor(0, 1); lcd.print("Unit : ug/m3");
      break;
    case 2:
      lcd.setCursor(0, 0); lcd.print("MQ2(ppm):"); lcd.print(norm_mq2, 1);
      lcd.setCursor(0, 1); lcd.print("MQ7(ppm):"); lcd.print(norm_mq7, 1);
      break;
    case 3:
      lcd.setCursor(0, 0); lcd.print("MQ135(ppm):"); lcd.print(norm_mq135, 1);
      lcd.setCursor(0, 1); lcd.print("Fan(%):"); lcd.print(pwm_percent);
      break;
  }
  screen = (screen + 1) % 4;

  // Serial monitor
  Serial.println("===== 📊 Normalized Sensor Readings =====");
  Serial.print("🌡 Temperature (°C): "); Serial.println(norm_temp, 2);
  Serial.print("💧 Humidity(%): "); Serial.println(norm_hum, 2);
  Serial.print("🔥 MQ2(Smoke/LPG): "); Serial.println(norm_mq2, 2);
  Serial.print("🧪 MQ7(CO): "); Serial.println(norm_mq7, 2);
  Serial.print("🌫 MQ135(Air Quality): "); Serial.println(norm_mq135, 2);
  Serial.print("🌁 PM2.5(ug/m3): "); Serial.println(dust, 2);
  Serial.print("Fan Speed(%): "); Serial.println(pwm_percent);
  Serial.println("=========================================");

  delay(2000);
}
