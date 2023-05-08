// queen cubator by matthias engelbracht

#include "ThingSpeak.h"
#include "secrets.h"
#include "DHT.h"
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <AutoPID.h>
#include <OneWire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//pins
#define OUT_IN1_PIN D5
#define OUT_IN2_PIN D6
#define POT_PIN A0
#define OUTPUT_PIN D7
#define LED_PIN D8
#define ONE_WIRE_BUS D4

#define TEMP_READ_DELAY 100 //can only read digital temp sensor every ~750ms

//pid settings and gains
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP 50    
#define KI 0.02  
#define KD 0     

unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int keyIndex = 0;

WiFiClient  client;

float kp = KP, ki = KI, kd = KD;

double temperature, temperature2, pressure, humidity, setPoint, outputVal;
bool heatRelayState = false;
bool temperaturAtSetPoint = false;
float pulseValue = 0;

Adafruit_BME280 bme;

//input/output variables passed by reference, so they are updated automatically
AutoPID temperaturPID(&temperature2, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, kp, ki, kd);

//unsigned long lastTempUpdate ; //tracks clock time of last temp update
unsigned long lastTime, lastThingsspeakUpdate, lastTempUpdate, thisTime = millis();
bool pidRun = true;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(9600);
  Serial.println("QueenCubator V5 2023-05-08");

  unsigned bmeStatus;

  bmeStatus = bme.begin(0x76);
  if (!bmeStatus) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    while (1) delay(10);
  }

  sensors.begin();

  pinMode(OUT_IN1_PIN, OUTPUT);
  pinMode(OUT_IN2_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(client);
  WiFi.begin(ssid, pass);
  //temperaturPID.setTimeStep(4000);
  temperaturPID.setBangBang(30, 37);
}

void readSensor() {
  if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
    temperature = bme.readTemperature();
    pressure = (bme.readPressure() / 100.0F);
    humidity = bme.readHumidity();
    //printSensorValues();

    sensors.requestTemperatures();
    temperature2 = sensors.getTempCByIndex(0);

    lastTempUpdate = millis();
  }
}

void loop() {
  if (Serial.available() > 0 ) {
    String str = Serial.readString();
    if (str.indexOf("run") > -1) {
      pidRun = true;
    }
    if (str.indexOf("stop") > -1) {
      pidRun = false;
      outputVal = 0;
      heatRelayState = false;
    }
  }

  readSensor();

  setPoint = 34.85; // set cubator temperatur

  if (pidRun == true) {
    temperaturPID.run(); //call every loop, updates automatically at certain time interval
  }
  else
  {
    temperaturPID.stop(); //call every loop, updates automatically at certain time interval
  }

  thisTime = millis();

  if (thisTime - lastTime > 5000) {
    Serial.print(temperature);
    Serial.print("; ");
    Serial.print(temperature2);
    Serial.print("; ");
    Serial.println(outputVal);
    lastTime = thisTime;
  }

  if (pidRun and (thisTime - lastThingsspeakUpdate > 30000)) {
    // Connect or reconnect to WiFi
    if (WiFi.status() != WL_CONNECTED) {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(SECRET_SSID);
      WiFi.begin(ssid, pass);
    }
    if (WiFi.status() == WL_CONNECTED) {
      //Serial.println("\nConnected.");

      if (not isnan(temperature)) {
        ThingSpeak.setField(1, float(temperature));
      }
      if (not isnan(temperature2)) {
        ThingSpeak.setField(4, float(temperature2));
      }
      if (not isnan(humidity)) {
        ThingSpeak.setField(2, float(humidity));
      }
      if (not isnan(outputVal)) {
        ThingSpeak.setField(3, float(outputVal));
      }

      int httpCode = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

      if (httpCode == 200) {
        //Serial.println("Channels write successful.");
      }
      else {
        Serial.println("Problem writing to channel. HTTP error code " + String(httpCode));
      }
      client.stop();
    }
    lastThingsspeakUpdate = thisTime;
  }
  analogWrite(OUTPUT_PIN, outputVal);

  temperaturAtSetPoint = temperaturPID.atSetPoint(1);
  digitalWrite(LED_PIN, temperaturAtSetPoint);
}
