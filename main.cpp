//include necessary libraries
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

//define a variable that stores the amount of pressure at sea level and is used to estimate altitude 
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C protocol definition

//defining the WiFi network to which microcontroller is connected
#define WIFISSID "…" // Put your WifiSSID here
#define PASSWORD "…" // Put your Wifi password here

//informations for accessing the MQTT broker
#define TOKEN "Put your token here" 
#define MQTT_CLIENT_NAME "Put your client name" 

//defining variables
#define VARIABLE_LABEL1 "Temperature" 
#define VARIABLE_LABEL2 "Humidity" 
#define VARIABLE_LABEL3 "Pressure"
#define VARIABLE_LABEL4 "Altitude"
#define VARIABLE_LABEL5 "Gas"
#define DEVICE_LABEL "ESP32"

char mqttBroker[] = "industrial.api.ubidots.com";
char payload[1000];
char topic1[150];
char topic2[150];
char topic3[150];
char topic4[150];
char topic5[150];

// Space to store values to send
char str_Temperature[10];
char str_Humidity[10];
char str_Pressure[10];
char str_Altitude[10];
char str_Gas[10];

//auxiliary functions
WiFiClient ubidots;
PubSubClient client(ubidots);

void callback(char* topic, byte* payload, unsigned int length){
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = 0;
  String message(p);
  Serial.write(payload, length);
  Serial.println(topic);
}

void reconnect(){
// Loop until we're reconnected

  while (!client.connected()) {
  Serial.println("Attempting MQTT connection...");
  // Attemp to connect
  if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")) {
    Serial.println("Connected");

  } else {
    Serial.print("Failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 2 seconds");
    // Wait 2 seconds before retrying
    delay(2000);
    }
  }
}

//main functions
void setup() {
  Serial.begin(115200);
  while (!Serial);
    Serial.println(F("BME680 test"));

  if (!bme.begin()){
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
  while (1);
  }

  // set resampling and filter initialization to increase sensor data resolution, default parameters are used 
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3); 
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  WiFi.begin(WIFISSID, PASSWORD);
  Serial.println();
  Serial.print("Waiting for WiFi Connection ..............");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqttBroker, 1883);
  client.setCallback(callback);
}

//read sensor data
void loop() {
  if (!client.connected()) {
    reconnect();
  }

  if (! bme.performReading()){
    Serial.println("Failed to perform reading :(");
    return;
  }

  float temperature = (bme.temperature);
  float humidity = (bme.humidity);
  float pressure = (bme.pressure / 100.0);
  float altitude = (bme.readAltitude(SEALEVELPRESSURE_HPA));
  float gas = (bme.gas_resistance / 1000.0);

  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(altitude);
  Serial.println(" m");

  Serial.print("Gas = ");
  Serial.print(gas);
  Serial.println(" Kohms");

  dtostrf(temperature, 4, 2, str_Temperature);
  dtostrf(humidity, 4, 2, str_Humidity);
  dtostrf(pressure, 4, 2, str_Pressure);
  dtostrf(altitude, 4, 2, str_Altitude);
  dtostrf(gas, 4, 2, str_Gas);

  //transfer data to MQTT broker
  sprintf(topic1, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  sprintf(payload, "%s", "");
  sprintf(payload, "{\"%s\":", VARIABLE_LABEL1);
  sprintf(payload, "%s {\"value\": %s}}", payload, str_Temperature);
  Serial.println("Publishing temperature to Ubidots Cloud");
  client.publish(topic1, payload);
 
  delay(100);
 
  sprintf(topic2, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  sprintf(payload, "%s", "");
  sprintf(payload, "{\"%s\":", VARIABLE_LABEL2);
  sprintf(payload, "%s {\"value\": %s}}", payload, str_Humidity);
  Serial.println("Publishing humidity to Ubidots Cloud");
  client.publish(topic2, payload);
 
  delay(100);
 
  sprintf(topic3, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  sprintf(payload, "%s", "");
  sprintf(payload, "{\"%s\":", VARIABLE_LABEL3);
  sprintf(payload, "%s {\"value\": %s}}", payload, str_Pressure);
  Serial.println("Publishing Pressure data to Ubidots Cloud");
  client.publish(topic3, payload);
 
  delay(100);
 
  sprintf(topic4, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  sprintf(payload, "%s", "");
  sprintf(payload, "{\"%s\":", VARIABLE_LABEL4);
  sprintf(payload, "%s {\"value\": %s}}", payload, str_Altitude);
  Serial.println("Publishing Altitude data to Ubidots Cloud");
  client.publish(topic4, payload);
  
  delay(100);
  
  sprintf(topic5, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  sprintf(payload, "%s", "");
  sprintf(payload, "{\"%s\":", VARIABLE_LABEL5);
  sprintf(payload, "%s {\"value\": %s}}", payload, str_Gas);
  Serial.println("Publishing Gas data to Ubidots Cloud");
  client.publish(topic5, payload);
  
  Serial.println();
  client.loop();
  delay(5000);
}