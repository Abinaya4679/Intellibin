#define BLYNK_TEMPLATE_ID "TMPL3Fg05gPZA"
#define BLYNK_TEMPLATE_NAME "Smart Dust Bin"
#define BLYNK_AUTH_TOKEN "c6gHjtqxbcTN7CrplHKI3NTahJo8r23k"


#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include "HX711.h"
#include <TinyGPS++.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

#define TRIG_PIN_A 25
#define ECHO_PIN_A 26
#define TRIG_PIN_B 32
#define ECHO_PIN_B 33
#define HX711_DT 4
#define HX711_SCK 5
#define SERVO_PIN 27
#define SDA 21
#define SCL 22
#define RXD2 16
#define TXD2 17
#define BUTTON_PIN 12
#define PWM_3v 14
#define GND 13
#define OPERATOR_BUTTON_PIN 23

#define BLYNK_PRINT Serial

const int doorOpenAngle = 0;
const int doorCloseAngle = 90;
const int servoDelay = 10;
const float emptyDistance = 12.22;
const float fullDistance = 1.0;
const String phoneNumber = "918015813768"; 
const String apikey = "2c1b6945-2472-4b86-998d";
const unsigned long HOLD_TIME = 5000;

LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo doorServo;
HX711 scale;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
WiFiManager wm;

char ssid[32];
char pass[32];

bool shouldSaveConfig = false;
bool portalRunning = false;
unsigned long buttonPressStart = 0;
bool binSetFull = false;
bool bitSetPrint = true;
bool enableCheck = true;
bool checkOperatorBtn = false;
unsigned long operatorButtonStart = 0;
float prevFill = -1;
float prevWeight = -1;
unsigned long manualOverrideTime = 0;

void openDoor();
void closeDoor();
float readDistance(int trigPin, int echoPin);
float readWeight();
float calculateFillPercentage(float dist);
void checkInsideSensors();
void sendDustbinFullAlert(float fill, double lat, double lng);
void checkResetButton();
void checkOperatorButton();
void saveCredentials(const char* newSSID, const char* newPass);
void readCredentials();
void saveConfigCallback();

void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  pinMode(PWM_3v, OUTPUT);
  pinMode(GND, OUTPUT);
  digitalWrite(PWM_3v, HIGH);
  digitalWrite(GND, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(OPERATOR_BUTTON_PIN, INPUT_PULLDOWN);

  Wire.begin(SDA, SCL);
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SMART DUST BIN");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(2000);

  doorServo.attach(SERVO_PIN);
  doorServo.write(doorCloseAngle);

  pinMode(TRIG_PIN_A, OUTPUT);
  pinMode(ECHO_PIN_A, INPUT);
  pinMode(TRIG_PIN_B, OUTPUT);
  pinMode(ECHO_PIN_B, INPUT);

  scale.begin(HX711_DT, HX711_SCK);
  scale.tare();

  gpsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  wm.setSaveConfigCallback(saveConfigCallback);
  readCredentials();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");
  lcd.setCursor(0, 1);
  lcd.print("Please wait...");

  bool connected = false;
  while (!connected) {
    WiFi.begin(ssid, pass);
    unsigned long wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 20000) {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WIFI Connected");
      lcd.setCursor(0, 1);
      lcd.print(WiFi.SSID());
    } else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WiFi Not Connected");
      for (int i = 8; i >= 0; i--) {
        lcd.setCursor(0, 1);
        lcd.printf("Retrying in: %d", i);
        delay(1000);
      }
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Connecting WiFi");
      lcd.setCursor(0, 1);
      lcd.print("Please wait...");
    }
  }

  delay(1000);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Blynk.syncVirtual(V2);

  unsigned long syncStart = millis();
  while (millis() - syncStart < 3000) {
    Blynk.run();
    delay(100);
  }

  checkResetButton();

  if (binSetFull) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Bin Status: FULL");
    lcd.setCursor(0, 1);
    lcd.print("Manually Set");
    return;
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Checking Sensors");
  lcd.setCursor(0, 1);
  lcd.print("Please wait...");
  delay(2000);
  checkInsideSensors();
}

BLYNK_WRITE(V2) {
  binSetFull = (param.asInt() == 1);
}

void loop() {
  Blynk.run();
  checkResetButton();
  checkOperatorButton();

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (WiFi.status() != WL_CONNECTED) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Reconnect");
    lcd.setCursor(0, 1);
    lcd.print("Please wait...");
    Serial.println("WiFi disconnected, attempting reconnect...");

    bool connected = false;
    while (!connected) {
      WiFi.begin(ssid, pass);
      unsigned long wifiStart = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 20000) {
        delay(500);
        Serial.print(".");
      }

      if (WiFi.status() == WL_CONNECTED) {
        connected = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("WiFi Connected");
        lcd.setCursor(0, 1);
        lcd.print(WiFi.SSID());
        delay(1000);
        lcd.setCursor(0, 1);
        lcd.printf("Full: %.1f%%       ", prevFill);
      } else {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("WiFi Not Connected");
        for (int i = 8; i >= 0; i--) {
          lcd.setCursor(0, 1);
          lcd.printf("Retrying in: %d", i);
          delay(1000);
        }
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("WiFi Reconnect");
        lcd.setCursor(0, 1);
        lcd.print("Please wait...");
      }
    }
  }

  if (binSetFull) {
    checkOperatorBtn = true;
    if (bitSetPrint) {
      lcd.setCursor(0, 0);
      lcd.print("Bin Status: FULL");
      lcd.setCursor(0, 1);
      lcd.print("Waiting Reset   ");
    }
    return;
  }

  static unsigned long lastCheck = 0;
  if (enableCheck && millis() - lastCheck >= 1000) {
    lastCheck = millis();
    float outsideDist = readDistance(TRIG_PIN_A, ECHO_PIN_A);

    if (outsideDist <= 10.0 && outsideDist > 0) {
      openDoor();
      lcd.setCursor(0, 0);
      lcd.print("Dustbin: Open   ");
      while (readDistance(TRIG_PIN_A, ECHO_PIN_A) <= 15.0) delay(100);
      closeDoor();
      lcd.setCursor(0, 0);
      lcd.print("Dustbin: Closed ");
      delay(1000);
      checkInsideSensors();
    } else {
      lcd.setCursor(0, 0);
      lcd.print("Dustbin: Closed ");
    }
  }
}

void checkResetButton() {
  if (digitalRead(BUTTON_PIN) == HIGH) {
    if (buttonPressStart == 0) {
      enableCheck = false;
      bitSetPrint = false;
      buttonPressStart = millis();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Hold to Reset...");
      lcd.setCursor(0, 1);
      lcd.print("WIFI SSID & Pass");
    } else if (!portalRunning && millis() - buttonPressStart >= HOLD_TIME) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WiFi Reset Mode");
      lcd.setCursor(0, 1);
      lcd.print("Starting Portal");

      WiFi.disconnect(true);
      wm.startConfigPortal("Smart DustBin Wifi_config");
      portalRunning = true;

      if (shouldSaveConfig) {
        saveCredentials(wm.getWiFiSSID().c_str(), wm.getWiFiPass().c_str());
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Restarting...");
        delay(1000);
        ESP.restart();
      }
    }
  } else {
    buttonPressStart = 0;
    if (!enableCheck) {
      enableCheck = true;
      bitSetPrint = true;
      if (binSetFull) {
        lcd.setCursor(0, 1);
        lcd.print("Waiting Reset   ");
      } else {
        lcd.setCursor(0, 1);
        lcd.printf("Full: %.1f%%      ", prevFill);
      }
    }
  }
}

void checkOperatorButton() {
  if (checkOperatorBtn) {
    static bool doorOpenedForReset = false;
    static bool actionTaken = false;
    bool currentState = digitalRead(OPERATOR_BUTTON_PIN);

    if (currentState == HIGH) {
      if (operatorButtonStart == 0) {
        operatorButtonStart = millis();
        lcd.clear();

        if (doorOpenedForReset) {
          lcd.setCursor(0, 0);
          lcd.print("Hold to Reset...");
          lcd.setCursor(0, 1);
          lcd.print("DUST BIN Value");
        } else {
          lcd.setCursor(0, 0);
          lcd.print("Hold to open");
          lcd.setCursor(0, 1);
          lcd.print("Dust Bin Door...");
        }
        bitSetPrint = false;
      }

      if (millis() - operatorButtonStart >= HOLD_TIME && !actionTaken) {
        if (!doorOpenedForReset) {
          openDoor();
          binSetFull = true;
          Blynk.virtualWrite(V2, 1);
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Door Opened     ");
          lcd.setCursor(0, 1);
          lcd.print("Marked as Full  ");
          doorOpenedForReset = true;
        } else {
          closeDoor();
          binSetFull = false;
          Blynk.virtualWrite(V0, 0.0);
          Blynk.virtualWrite(V2, 0);
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Door Closed     ");
          lcd.setCursor(0, 1);
          lcd.print("System Reset    ");
          delay(2000);
          lcd.clear();
          lcd.setCursor(0, 1);
          lcd.print("Full: 0.0%       ");
          doorOpenedForReset = false;
        }
        actionTaken = true;
      }
    } else {
      operatorButtonStart = 0;
      if (!doorOpenedForReset) bitSetPrint = true;
      actionTaken = false;
    }
  }
}

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;
}

float readWeight() {
  if (scale.is_ready()) {
    long raw = scale.get_units(10);
    return raw / 1000.0;
  }
  return 0;
}

float calculateFillPercentage(float dist) {
  if (dist <= fullDistance) return 100.0;
  if (dist >= emptyDistance) return 0.0;
  return ((emptyDistance - dist) / (emptyDistance - fullDistance)) * 100.0;
}

void openDoor() {
  for (int pos = doorCloseAngle; pos >= doorOpenAngle; pos--) {
    doorServo.write(pos);
    delay(servoDelay);
  }
}

void closeDoor() {
  for (int pos = doorOpenAngle; pos <= doorCloseAngle; pos++) {
    doorServo.write(pos);
    delay(servoDelay);
  }
}

void checkInsideSensors() {
  float insideDist = readDistance(TRIG_PIN_B, ECHO_PIN_B);
  float weight = readWeight();
  float fill = calculateFillPercentage(insideDist);

  Serial.printf("Inside Distance: %.2f cm\nWeight: %.3f kg\nFill: %.1f%%\n", insideDist, weight, fill);
  lcd.setCursor(0, 1);
  lcd.printf("Full: %.1f%%     ", fill);

  if (abs(fill - prevFill) >= 1.0) {
    Blynk.virtualWrite(V0, fill);
    prevFill = fill;
  }

  if (abs(weight - prevWeight) >= 0.1) {
    Blynk.virtualWrite(V1, weight);
    prevWeight = weight;
  }

  if (fill >= 95.0 && !binSetFull) {
    Blynk.virtualWrite(V2, 1);
    if(gps.location.isValid())
      sendDustbinFullAlert(fill, weight, gps.location.lat(), gps.location.lng());
    else 
      sendDustbinFullAlert(fill, weight);
      binSetFull = true;
  } 
}

void sendDustbinFullAlert(float fill, float weight) {
    String msg = "🚮 Dustbin is " + String(fill, 1) + "% Full! & Weight load is "+ String(weight,2);
    sendMessage(msg);-/
}

void sendDustbinFullAlert(float fill, float weight, double lat, double lng) {
    String maps = "https://maps.google.com/?q=" + String(lat, 6) + "," + String(lng, 6);
    String msg = "🚮 Dustbin is " + String(fill, 1) + "% Full! & Weight load is "+ String(weight,2) +" mg \n📍 Location: " + maps;
    sendMessage(msg);
}

void sendMessage(const String &text) {
  if (Wi  Fi.status() == WL_CONNECTED) {
    const String url = "https://api.whatabot.io/Whatsapp/RequestSendMessage";
    HTTPClient http;
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-api-key", apikey);

    StaticJsonDocument<512> doc;
    doc["ApiKey"] = apikey;
    doc["Phone"] = phoneNumber;
    doc["Text"] = text;

    String requestBody;
    serializeJson(doc, requestBody);
    int httpCode = http.POST(requestBody);
    if (httpCode == 200) {
      Serial.println("✅ Message sent successfully!");
    } else {
      Serial.print("❌ HTTP Error ");
      Serial.println(httpCode);
      Serial.println(http.getString());
    }
    http.end();
  } else {
    Serial.println("WiFi not connected.");
  }
}

void saveCredentials(const char* newSSID, const char* newPass) {
  for (int i = 0; i < 32; i++) {
    EEPROM.write(i, newSSID[i]);
    EEPROM.write(100 + i, newPass[i]);
  }
  EEPROM.commit();
}

void readCredentials() {
  for (int i = 0; i < 32; i++) {
    ssid[i] = EEPROM.read(i);
    pass[i] = EEPROM.read(100 + i);
  }
  ssid[31] = '\0';
  pass[31] = '\0';
}

void saveConfigCallback() {
  shouldSaveConfig = true;
}
