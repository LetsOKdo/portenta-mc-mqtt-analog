/***********************************************************************
  Portenta Machine Control client controlling analogue outputs
  using a remote CO2 sensor over WiFi / MQTT

  Version: 0.1
  Date: 15 February 2022
  Author: Peter Milne

  Copywrite 2022 Peter Milne
  Released under GNU GENERAL PUBLIC LICENSE
  Version 3, 29 June 2007

 **********************************************************************/

#include <Arduino_MachineControl.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "ArduinoJson.h"
#include "arduino_secrets.h"

using namespace machinecontrol;

// Maximum analog output value is 10.4V
#define MAX_VOLTAGE 10

// Analog channel AO0
#define CHANNEL 0

// Min & Max CO2 tolerances
#define MIN_CO2 800
#define MAX_CO2 2000

// Set according to MQTT message size
#define BUFFER_SIZE 512

// IP address or hostname depending on network
const char server[] = "192.168.1.104";
//const char server[] = "airquality";
int port = 1883;
const char topic[] = "airquality/#";

/////// Enter sensitive data in arduino_secrets.h
char ssid[] = SECRET_SSID;  // Network SSID
char pass[] = SECRET_PASS;  // WPA key

int co2 = 0;
int previous_co2 = 0;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  Serial.begin(115200);

  //analog_out.period_ms(CHANNEL, PERIOD_MILLISECONDS);
  analog_out.period_ms(CHANNEL, 4);

  // Configure MQTT client
  mqttClient.setServer(server, port);
  mqttClient.setCallback(callback);
  mqttClient.setBufferSize(BUFFER_SIZE);

  delay(10000);
  Serial.println("STARTING");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    // Wifi.begin blocks until connect or failure timeout
    WiFi.begin(ssid, pass);
    delay(10000);
    printWiFiStatus(WiFi.status());
  }

  if (!mqttClient.connected()) {
    connectMQTT();
    printMQTTStatus(mqttClient.state());
  }

  mqttClient.loop();

  if (co2 != previous_co2) {
    previous_co2 = co2;
    float input = mapInput(co2, MIN_CO2, MAX_CO2);
    float volts = updateAnalogOut(CHANNEL, input);

    Serial.print(co2);
    Serial.print(",");
    Serial.println(volts);
  }
}

// Map input to 0 - 10 volts
float mapInput(long input, long min_in, long max_in) {
  if (input < min_in) {
    input = min_in;
  }
  if (input > max_in) {
    input = max_in;
  }
  return (float)map(input, min_in, max_in, 0, MAX_VOLTAGE);
}

// Update analogue output
float updateAnalogOut(int channel, float input) {
  if (input < 0) {
    input = 0;
  }
  if (input > MAX_VOLTAGE) {
    input = MAX_VOLTAGE;
  }
  float volts = input;
  //analog_out.write(CHANNEL, OUTPUT_VOLTAGE_VALUE);
  analog_out.write(channel, volts);
  return volts;
}

void connectMQTT() {
  if ((WiFi.status() == WL_CONNECTED) && (!mqttClient.connected())) {
    if (!mqttClient.connect("MCP01")) {
      // Portenta needs to force a disconnect
      mqttClient.disconnect();
      delay(10000);
    }
    else {
      mqttClient.subscribe(topic);
    }
  }
}

void callback(char* topic, byte * payload, unsigned int length) {
  StaticJsonDocument<BUFFER_SIZE> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.println(error.c_str());
    return;
  }
  co2 = doc["co2"]["co2"];
}

void printWiFiStatus(int state) {
  switch (state) {
    case WL_IDLE_STATUS:
      Serial.println("WL_IDLE_STATUS");
      break;
    case WL_NO_SSID_AVAIL:
      Serial.println("WL_NO_SSID_AVAIL");
      break;
    case WL_SCAN_COMPLETED:
      Serial.println("WL_SCAN_COMPLETED");
      break;
    case WL_CONNECTED:
      Serial.println("WL_CONNECTED");
      break;
    case WL_CONNECT_FAILED:
      Serial.println("WL_CONNECT_FAILED");
      break;
    case WL_CONNECTION_LOST:
      Serial.println("WL_CONNECTION_LOST");
      break;
    case WL_DISCONNECTED:
      Serial.println("WL_DISCONNECTED");
  }
}

void printMQTTStatus(int state) {
  switch (state) {
    case MQTT_CONNECTION_TIMEOUT:
      Serial.println("MQTT_CONNECTION_TIMEOUT");
      break;
    case MQTT_CONNECTION_LOST:
      Serial.println("MQTT_CONNECTION_LOST");
      break;
    case MQTT_CONNECT_FAILED:
      Serial.println("MQTT_CONNECT_FAILED");
      break;
    case MQTT_DISCONNECTED:
      Serial.println("MQTT_DISCONNECTED");
      break;
    case MQTT_CONNECTED:
      Serial.println("MQTT_CONNECTED");
      break;
    case MQTT_CONNECT_BAD_PROTOCOL:
      Serial.println("MQTT_CONNECT_BAD_PROTOCOL");
      break;
    case MQTT_CONNECT_BAD_CLIENT_ID:
      Serial.println("MQTT_CONNECT_BAD_CLIENT_ID");
      break;
    case MQTT_CONNECT_UNAVAILABLE:
      Serial.println("MQTT_CONNECT_UNAVAILABLE");
      break;
    case MQTT_CONNECT_BAD_CREDENTIALS:
      Serial.println("MQTT_CONNECT_BAD_CREDENTIALS");
      break;
    case MQTT_CONNECT_UNAUTHORIZED:
      Serial.println("MQTT_CONNECT_UNAUTHORIZED");
  }
}
