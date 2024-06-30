#include <Wire.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

#define ANALOG_IN_PIN A3
const int sensor = A0;

float adc_voltage = 0.0;
float in_voltage = 0.0;

float R1 = 30000.0;
float R2 = 7500.0;
float current;
float avgVoltage;
float ref_voltage = 5.0;

int Tracker_1 = 0;
int Tracker_2 = 0;
int Tracker_3 = 0;
int Tracker_4 = 0;

const int currentSensorPin = A2;
const float sensorOffset = 2.5;
const float sensitivity = 184;
const int numReadings = 100;

int adc_value = 0;
int Relay = 7;

float tempc;

float tempf;

float vout;
enum State{
  IDLE,
  HIGH_TEMP,
  OVER_VOLTAGE,
  UNDER_VOLTAGE,
  HIGH_CURRENT
};

State currentState = IDLE;

SoftwareSerial mySerial(5, 6);

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);

  pinMode(Relay, OUTPUT);
  pinMode(sensor, INPUT);

  // mySerial.println("AT");
  // updateSerial();
  mySerial.println("AT");  //Once the handshake test is successful, it will back to OK
  updateSerial();

  digitalWrite(Relay, HIGH);

  lcd.init();
  lcd.backlight();

  lcd.clear();
  lcd.setCursor(5, 0);
  lcd.print("SYSTEM");
  lcd.setCursor(1, 1);
  lcd.print("INITIALIZATION");
  delay(2000);
}

void loop() {
  voltagesensor();
  currentsensor();
  temperatureSensor();

  currentState = IDLE;

  if (tempc > 40) {
    currentState = HIGH_TEMP;
  } else if (in_voltage > 10.0 && tempc < 40) {
    currentState = OVER_VOLTAGE;
  } else if (in_voltage < 2.5 && current > 0.35) {
    currentState = HIGH_CURRENT;
  } else if (in_voltage >= 2.5 && in_voltage < 4.5) {
    currentState = UNDER_VOLTAGE;
  }

  switch (currentState) {
    case HIGH_TEMP:
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("ALERT");
      lcd.setCursor(0, 1);
      lcd.print("HIGH TEMPERATURE");
      if (Tracker_3 == 0) {
        sendAlert_Temperature();
        Tracker_1 = 0;
        Tracker_2 = 0;
        Tracker_3 = 1;
        Tracker_4 = 0;
      } else {
        Serial.println("Message Sent");
      }
      break;
    case OVER_VOLTAGE:
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("ALERT");
      lcd.setCursor(2, 1);
      lcd.print("OVER VOLTAGE");
      if (Tracker_2 == 0) {
        sendAlert_Overvoltage();
        Tracker_1 = 0;
        Tracker_2 = 1;
        Tracker_3 = 0;
        Tracker_4 = 0;
      } else {
        Serial.println("Message Sent");
      }
      break;
    case HIGH_CURRENT:
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("ALERT");
      lcd.setCursor(2, 1);
      lcd.print("HIGH CURRENT");
      if (Tracker_4 == 0) {
        sendAlert_HighCurrent();
        Tracker_1 = 0;
        Tracker_2 = 0;
        Tracker_3 = 0;
        Tracker_4 = 1;
      } else {
        Serial.println("Message Sent");
      }
      break;
    case UNDER_VOLTAGE:
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("ALERT");
      lcd.setCursor(1, 1);
      lcd.print("UNDER VOLTAGE");
      if (Tracker_1 == 0) {
        sendAlert_Undervoltage();
        Tracker_1 = 1;
        Tracker_2 = 0;
        Tracker_3 = 0;
        Tracker_4 = 0;
      } else {
        Serial.println("Message Sent");
      }
      break;
    case IDLE:
    default:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("V: ");
      lcd.print(in_voltage, 1);
      lcd.setCursor(8, 0);
      lcd.print("C: ");
      lcd.print(current, 2);
      lcd.print(" A");
      lcd.setCursor(0, 1);
      lcd.print("T: ");
      lcd.print(tempc, 1);
      lcd.print((char)223);
      lcd.print("C");
      //delay(1000);
      voltagesensor();
      currentsensor();
      temperatureSensor();
      break;
  }
  delay(1000);
}
void voltagesensor() {
  adc_value = analogRead(ANALOG_IN_PIN);
  adc_voltage = (adc_value * ref_voltage) / 1024.0;
  in_voltage = adc_voltage * (R1 + R2) / R2;
  // Serial.print("Input Voltage = ");
  // Serial.println(in_voltage, 2);
}
void temperatureSensor() {
  vout = analogRead(sensor);

  vout = (vout * 500) / 1023;
  //vout = vout / 10;

  tempc = vout;

  tempf = (vout * 1.8) + 32;

  delay(500);
}
void currentsensor() {
  float totalVoltage = 0;
  for (int i = 0; i < numReadings; i++) {
    int sensorValue = analogRead(currentSensorPin);
    float voltage = sensorValue * (5.0 / 1023.0);
    totalVoltage += voltage;
    delay(10);
  }

  avgVoltage = totalVoltage / numReadings;

  Serial.println("avVoltage: ");
  Serial.print(avgVoltage);
  Serial.print(" V");

  current = (avgVoltage - sensorOffset) / (sensitivity / 1000);

  if (current < 0) {
    current = current * -1;
  }
  delay(1000);
}
void updateSerial() {
  delay(500);
  while (Serial.available()) {
    mySerial.write(Serial.read());
  }
  while (mySerial.available()) {
    Serial.write(mySerial.read());
  }
}
void sendAlert_Undervoltage() {
  mySerial.println("AT+CMGF=1");
  mySerial.println("AT+CMGS=\"+254748613509\"");
  updateSerial();
  mySerial.print("ALert!! Under Voltage Detected");  //text content
  updateSerial();
  mySerial.write(26);
}

void sendAlert_Overvoltage() {
  mySerial.println("AT+CMGF=1");
  mySerial.println("AT+CMGS=\"+254748613509\"");
  updateSerial();
  mySerial.print("ALert!! Over Voltage Detected");  //text content
  updateSerial();
  mySerial.write(26);
}
void sendAlert_HighCurrent() {
  mySerial.println("AT+CMGF=1");
  mySerial.println("AT+CMGS=\"+254748613509\"");
  updateSerial();
  mySerial.print("ALert!! High Current Detected");  //text content
  updateSerial();
  mySerial.write(26);
}
void sendAlert_Temperature() {
  mySerial.println("AT+CMGF=1");
  mySerial.println("AT+CMGS=\"+254748613509\"");
  updateSerial();
  mySerial.print("ALert!! High Temperature Detected");  //text content
  updateSerial();
  mySerial.write(26);
}
