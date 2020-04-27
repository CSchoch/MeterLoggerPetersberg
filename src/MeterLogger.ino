/*
   Program for isolating and publishing smartmeter modbus data to a mqtt broker by using a esp32.
*/

#define LED_PIN 2
#define ENERGY_RX_PIN 14 // for NodeMCU: GPIO4 = D1
#define ENERGY_TX_PIN 26 // for NodeMCU: GPIO5 = D2
#define ENERGY_ADR 1     // Modbus adress
#define DEBUGLEVEL DEBUG

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 10       /* Time ESP32 will go to sleep (in seconds) */

#include <WiFi.h>
#include <ArduinoOTA.h>

#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_esp8266/mqtt_esp8266.ino
#include <HardwareSerial.h>
#include <ModbusMaster.h>
#include <DebugUtils.h>
#include <ArduinoJson.h>

//#include "Config.h" // make your own config file or remove this line and use the following lines
const char *clientId = "Energy";
const char *mqtt_server = "192.168.1.35";
#include "WifiCredentials.h"       // const char* ssid = "MySSID"; const char* WifiPassword = "MyPw";
#include "OTACredentials.h"        // const char* OtaPassword = "MyPw";
IPAddress ip(192, 168, 2, 6);      // Static IP
IPAddress dns(192, 168, 2, 1);     // most likely your router
IPAddress gateway(192, 168, 2, 1); // most likely your router
IPAddress subnet(255, 255, 255, 0);
unsigned long lastUpdated;
unsigned long lastLed;
uint8_t stateLed = HIGH;
boolean updateActive;
RTC_DATA_ATTR unsigned long bootCount;
RTC_DATA_ATTR boolean enableUpdate;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void update_led(unsigned long intervallHigh, unsigned long intervallLow)
{
  if (stateLed == HIGH && millis() - lastLed >= intervallHigh)
  {
    stateLed = LOW;
    lastLed = millis();
  }
  else if (stateLed == LOW && millis() - lastLed >= intervallLow)
  {
    stateLed = HIGH;
    lastLed = millis();
  }
  digitalWrite(LED_PIN, stateLed);
}

void setup_wifi()
{
  delay(10);
  DEBUGPRINTNONE("Connecting to ");
  DEBUGPRINTLNNONE(ssid);
  WiFi.config(ip, dns, gateway, subnet);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, WifiPassword);
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    DEBUGPRINTNONE(".");
    counter++;
    unsigned long timer = millis();
    while (millis() - timer <= 500)
    {
      update_led(100, 100);
    }
    //delay(500);
    if (counter >= 20)
    {
      counter = 0;
      DEBUGPRINTLNNONE("Retry");
      WiFi.disconnect();
      while (WiFi.status() == WL_CONNECTED)
      {
        DEBUGPRINTNONE(".");
        update_led(100, 100);
        delay(10);
      }
      WiFi.begin(ssid, WifiPassword);
    }
  }
  DEBUGPRINTLNNONE("WiFi connected");
  DEBUGPRINTNONE("IP address: ");
  DEBUGPRINTLNNONE(WiFi.localIP());
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  char *topic1 = "/enableUpdate";
  char *path = (char *)malloc(1 + strlen(clientId) + strlen(topic1));
  strcpy(path, clientId);
  strcat(path, topic1);
  //if (topic == path) {
  DEBUGPRINTLNNONE("NewMessage");
  DEBUGPRINTLNNONE(topic);
  DEBUGPRINTLNNONE(length);
  free(path);
  enableUpdate = true;
  topic = "/enableUpdateAck";
  path = (char *)malloc(1 + strlen(clientId) + strlen(topic));
  strcpy(path, clientId);
  strcat(path, topic);
  mqttClient.publish(path, "true");
  free(path);
  delay(100);
  //}
}

void mqttReconnect()
{
  // Loop until we're reconnected
  while (!mqttClient.connected())
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      wifiReconnect();
    }
    DEBUGPRINTNONE("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(clientId))
    {
      DEBUGPRINTLNNONE("connected");
      // ... and resubscribe
      char *topic = "/enableUpdate";
      char *path = (char *)malloc(1 + strlen(clientId) + strlen(topic));
      strcpy(path, clientId);
      strcat(path, topic);
      DEBUGPRINTLNNONE(path);
      mqttClient.subscribe(path);
      free(path);
    }
    else
    {
      DEBUGPRINTNONE("failed, rc=");
      DEBUGPRINTNONE(mqttClient.state());
      DEBUGPRINTLNNONE(" try again in 1 seconds");
      // Wait 5 seconds before retrying
      unsigned long timer = millis();
      while (millis() - timer <= 1000)
      {
        update_led(250, 250);
      }
      //delay(1000);
    }
  }
}

void wifiReconnect()
{
  WiFi.disconnect();
  while (WiFi.status() == WL_CONNECTED)
  {
    DEBUGPRINTNONE(".");
    delay(10);
  }
  DEBUGPRINTNONE("Reconnecting to ");
  DEBUGPRINTLNNONE(ssid);
  int counter = 0;
  WiFi.begin(ssid, WifiPassword);
  while (WiFi.status() != WL_CONNECTED and counter < 20)
  {
    DEBUGPRINTNONE(".");
    counter++;
    unsigned long timer = millis();
    while (millis() - timer <= 500)
    {
      update_led(100, 100);
    }
    //delay(500);
  }
  DEBUGPRINTLNNONE("WiFi connected");
  DEBUGPRINTNONE("IP address: ");
  DEBUGPRINTLNNONE(WiFi.localIP());
}

// void setup() {
//   bootCount ++;
//   updateActive = enableUpdate;
//   lastUpdated = millis();
//   lastLed = millis();
//   Serial.begin(115200);
//   pinMode(LED_PIN, OUTPUT);
//   DEBUGPRINTLNNONE("\nHardware serial started");
//   DEBUGPRINTLNNONE(bootCount);
//   Serial1.begin(9600, SERIAL_8N1, ENERGY_RX_PIN, ENERGY_TX_PIN);
//   EnergyMeter.begin(ENERGY_ADR, Serial1);
//   DEBUGPRINTLNNONE("\nEnergy meter serial started");
//   setup_wifi();
//   mqttClient.setServer(mqtt_server, 1883);
//   mqttClient.setCallback(mqttCallback);
//   // --------------------------------------------------------------------- OTA

//   // Port defaults to 8266
//   if (enableUpdate) {
//     ArduinoOTA.setPort(8266);

//     // Hostname defaults to esp8266-[ChipID]
//     ArduinoOTA.setHostname(clientId);

//     // No authentication by default
//     ArduinoOTA.setPassword(OtaPassword);

//     ArduinoOTA.onStart([]() {
//       Serial.println("Start");
//     });
//     ArduinoOTA.onEnd([]() {
//       enableUpdate = false;
//       updateActive = false;
//       DEBUGPRINTDEBUG("WiFi.disconnect");
//       WiFi.disconnect();
//       while (WiFi.status() == WL_CONNECTED) {
//         DEBUGPRINTDEBUG('.');
//       }
//       DEBUGPRINTLNDEBUG("");
//       Serial.println("\nEnd");
//     });
//     ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
//       Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
//       Serial.println("");
//     });
//     ArduinoOTA.onError([](ota_error_t error) {
//       Serial.printf("Error[%u]: ", error);
//       if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
//       else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
//       else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
//       else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
//       else if (error == OTA_END_ERROR) Serial.println("End Failed");
//     });
//     ArduinoOTA.begin();
//   }
// }

// void loop() {
//   char Data[256];
//   ArduinoOTA.handle();
//   if (!mqttClient.connected()) {
//     mqttReconnect();
//   }
//   mqttClient.loop();

//   OutfeedMeter.loop();
//   SolarMeter.loop();
//   if (millis() - lastUpdated >= 10000 && OutfeedMeter.getAvailable() && SolarMeter.getAvailable()) {
//     OutfeedMeter.resetAvailable();
//     SolarMeter.resetAvailable();
//     double value = SolarMeter.getTotalSupply() - OutfeedMeter.getTotalSupply();
//     DEBUGPRINTNONE("Eigenverbrauch: ");
//     DEBUGPRINTNONE(value);
//     DEBUGPRINTNONE("kWh ");
//     value = (value / SolarMeter.getTotalSupply()) * 100;
//     DEBUGPRINTNONE(value);
//     DEBUGPRINTLNNONE("%");

//     value = SolarMeter.getActualPower();
//     DEBUGPRINTNONE("Erzeugung: ");
//     DEBUGPRINTNONE(value, 0);
//     DEBUGPRINTLNNONE("W");

//     value = OutfeedMeter.getActualPower();
//     DEBUGPRINTNONE("Bezug: ");
//     DEBUGPRINTNONE(value, 0);
//     DEBUGPRINTLNNONE("W");

//     value = OutfeedMeter.getActualPower() - SolarMeter.getActualPower();
//     DEBUGPRINTNONE("Hausverbrauch: ");
//     DEBUGPRINTNONE(value, 0);
//     DEBUGPRINTLNNONE("W");

//     const size_t capacity = JSON_OBJECT_SIZE(2) + 4*JSON_OBJECT_SIZE(3);
//     DynamicJsonDocument doc(capacity);

//     JsonObject Outfeed = doc.createNestedObject("Outfeed");
//     sprintf(Data, "%ld", OutfeedMeter.getActualPower());
//     Outfeed["actualPower"] = Data;
//     sprintf(Data, "%lf", OutfeedMeter.getTotalConsumption());
//     Outfeed["totalConsumption"] = Data;
//     sprintf(Data, "%lf", OutfeedMeter.getTotalSupply());
//     Outfeed["totalSupply"] = Data;

//     JsonObject Solar = doc.createNestedObject("Solar");
//     sprintf(Data, "%ld", SolarMeter.getActualPower());
//     Solar["actualPower"] = Data;
//     sprintf(Data, "%lf", SolarMeter.getTotalConsumption());
//     Solar["totalConsumption"] = Data;
//     sprintf(Data, "%lf", SolarMeter.getTotalSupply());
//     Solar["totalSupply"] = Data;

//     serializeJson(doc, Data, sizeof(Data));
//     char* topic = "/MeterData";
//     char* path = (char *)malloc(1 + strlen(clientId) + strlen(topic));
//     strcpy(path, clientId);
//     strcat(path, topic);
//     if (!mqttClient.publish(path, Data, true)){
//       DEBUGPRINTLNNONE("MQTT publish failed");
//     }
//     free(path);

//     DEBUGPRINTDEBUG(topic);
//     DEBUGPRINTDEBUG(" ");
//     DEBUGPRINTLNDEBUG(Data);

//     delay(100);
//     update_led(500, 500);

//     lastUpdated = millis();
//   }
//   else if (millis() - lastUpdated < 500) {
//     update_led(500, 500);
//   }
//   else{
//     update_led(100, 1000);
//   }

//   if ((!updateActive && enableUpdate) || (millis() - lastUpdated >= 60000))  {

//     DEBUGPRINTLNDEBUG("MQTT disconnect");
//     mqttClient.disconnect();
//     while (mqttClient.connected()) {
//       DEBUGPRINTDEBUG(".");
//     }
//     DEBUGPRINTLNDEBUG("");

//     DEBUGPRINTDEBUG("WiFi.disconnect");
//     WiFi.disconnect();
//     while (WiFi.status() == WL_CONNECTED) {
//       DEBUGPRINTDEBUG(".");
//     }
//     DEBUGPRINTLNDEBUG("");

//     DEBUGPRINTLNDEBUG("Sleep");

//     esp_sleep_enable_timer_wakeup(1 * uS_TO_S_FACTOR);
//     delay(1 * 1000);
//     esp_deep_sleep_start();
//   }
// }

/*

  Basic.pde - example using ModbusMaster library

  Library:: ModbusMaster
  Author:: Doc Walker <4-20ma@wvfans.net>

  Copyright:: 2009-2016 Doc Walker

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

*/

#include <OR_WE.h>

// instantiate ModbusMaster object
OR_WE EnergyMeter;

void setup()
{
  Serial.begin(115200);
  DEBUGPRINTLNNONE("\nHardware serial started");
  DEBUGPRINTLNNONE(bootCount);
  Serial1.begin(OR_WE_SERIAL_BAUD, OR_WE_SERIAL_MODE, ENERGY_RX_PIN, ENERGY_TX_PIN);

  // communicate with Modbus slave ID 1 over Serial (port 1)
  EnergyMeter.begin(Serial1, 1);
}

float getModbusFloat(uint16_t data[2])
{
  union u_data {
    byte b[4];
    uint16_t data[2];
  } source;

  union u_tag {
    byte b[4];
    float fval;
  } dest;

  source.data[0] = data[0];
  source.data[1] = data[1];

  dest.b[2] = source.b[0];
  dest.b[3] = source.b[1];
  dest.b[0] = source.b[2];
  dest.b[1] = source.b[3];

  return dest.fval;
}

void loop()
{
  DEBUGPRINTLNNONE("Voltage");
  DEBUGPRINTNONE("L1: ");
  DEBUGPRINTNONE(EnergyMeter.getVoltageL1());

  DEBUGPRINTNONE(" L2: ");
  DEBUGPRINTNONE(EnergyMeter.getVoltageL2());

  DEBUGPRINTNONE(" L3: ");
  DEBUGPRINTLNNONE(EnergyMeter.getVoltageL3());

  DEBUGPRINTNONE("Freq: ");
  DEBUGPRINTLNNONE(EnergyMeter.getFrequency());

  //Current
  DEBUGPRINTLNNONE("Current");
  DEBUGPRINTNONE("L1: ");
  DEBUGPRINTNONE(EnergyMeter.getCurrentL1());
  DEBUGPRINTNONE(" L2: ");
  DEBUGPRINTNONE(EnergyMeter.getCurrentL2());
  DEBUGPRINTNONE(" L3: ");
  DEBUGPRINTLNNONE(EnergyMeter.getCurrentL3());

  //Active Power
  DEBUGPRINTLNNONE("ActivePower");
  DEBUGPRINTNONE("Total ");
  DEBUGPRINTNONE(EnergyMeter.getActivePowerTotal());
  DEBUGPRINTNONE(" L1: ");
  DEBUGPRINTNONE(EnergyMeter.getActivePowerL1());
  DEBUGPRINTNONE(" L2: ");
  DEBUGPRINTNONE(EnergyMeter.getActivePowerL2());
  DEBUGPRINTNONE(" L3: ");
  DEBUGPRINTLNNONE(EnergyMeter.getActivePowerL3());

  //Reactive Power
  DEBUGPRINTLNNONE("Reactive Power");
  DEBUGPRINTNONE("Total: ");
  DEBUGPRINTNONE(EnergyMeter.getReactivePowerTotal());
  DEBUGPRINTNONE(" L1: ");
  DEBUGPRINTNONE(EnergyMeter.getReactivePowerL1());
  DEBUGPRINTNONE(" L2: ");
  DEBUGPRINTNONE(EnergyMeter.getReactivePowerL2());
  DEBUGPRINTNONE(" L3: ");
  DEBUGPRINTLNNONE(EnergyMeter.getReactivePowerL3());

  //Apparent Power
  DEBUGPRINTLNNONE("Apparent Power");
  DEBUGPRINTNONE("Total: ");
  DEBUGPRINTNONE(EnergyMeter.getApparentPowerTotal());
  DEBUGPRINTNONE(" L1: ");
  DEBUGPRINTNONE(EnergyMeter.getApparentPowerL1());
  DEBUGPRINTNONE(" L2: ");
  DEBUGPRINTNONE(EnergyMeter.getApparentPowerL2());
  DEBUGPRINTNONE(" L3: ");
  DEBUGPRINTLNNONE(EnergyMeter.getApparentPowerL3());

  //Power Factor
  DEBUGPRINTLNNONE("Power Factor");
  DEBUGPRINTNONE("Total: ");
  DEBUGPRINTNONE(EnergyMeter.getPowerFactorTotal());
  DEBUGPRINTNONE(" L1: ");
  DEBUGPRINTNONE(EnergyMeter.getPowerPowerFactorL1());
  DEBUGPRINTNONE(" L2: ");
  DEBUGPRINTNONE(EnergyMeter.getPowerPowerFactorL2());
  DEBUGPRINTNONE(" L3: ");
  DEBUGPRINTLNNONE(EnergyMeter.getPowerPowerFactorL3());

  //Counter
  DEBUGPRINTLNNONE("Counter Active Power");
  DEBUGPRINTNONE("Total: ");
  DEBUGPRINTNONE(EnergyMeter.getTotalCounterActivePowerTotal());
  DEBUGPRINTNONE(" L1: ");
  DEBUGPRINTNONE(EnergyMeter.getTotalCounterActivePowerL1());
  DEBUGPRINTNONE(" L2: ");
  DEBUGPRINTNONE(EnergyMeter.getTotalCounterActivePowerL2());
  DEBUGPRINTNONE(" L3: ");
  DEBUGPRINTLNNONE(EnergyMeter.getTotalCounterActivePowerL3());

  DEBUGPRINTLNNONE("Counter Reactive Power");
  DEBUGPRINTNONE("Total: ");
  DEBUGPRINTNONE(EnergyMeter.getTotalCounterReactivePowerTotal());
  DEBUGPRINTNONE(" L1: ");
  DEBUGPRINTNONE(EnergyMeter.getTotalCounterReactivePowerL1());
  DEBUGPRINTNONE(" L2: ");
  DEBUGPRINTNONE(EnergyMeter.getTotalCounterReactivePowerL2());
  DEBUGPRINTNONE(" L3: ");
  DEBUGPRINTLNNONE(EnergyMeter.getTotalCounterReactivePowerL3());

  //Counter Import
  DEBUGPRINTLNNONE("Counter Import Active Power");
  DEBUGPRINTNONE("Total: ");
  DEBUGPRINTNONE(EnergyMeter.getImportCounterActivePowerTotal());
  DEBUGPRINTNONE(" L1: ");
  DEBUGPRINTNONE(EnergyMeter.getImportCounterActivePowerL1());
  DEBUGPRINTNONE(" L2: ");
  DEBUGPRINTNONE(EnergyMeter.getImportCounterActivePowerL2());
  DEBUGPRINTNONE(" L3: ");
  DEBUGPRINTLNNONE(EnergyMeter.getImportCounterActivePowerL3());

  DEBUGPRINTLNNONE("Counter Import Reactive Power");
  DEBUGPRINTNONE("Total: ");
  DEBUGPRINTNONE(EnergyMeter.getImportCounterReactivePowerTotal());
  DEBUGPRINTNONE(" L1: ");
  DEBUGPRINTNONE(EnergyMeter.getImportCounterReactivePowerL1());
  DEBUGPRINTNONE(" L2: ");
  DEBUGPRINTNONE(EnergyMeter.getImportCounterReactivePowerL2());
  DEBUGPRINTNONE(" L3: ");
  DEBUGPRINTLNNONE(EnergyMeter.getImportCounterReactivePowerL3());

  //Counter Export
  DEBUGPRINTLNNONE("Counter Export Active Power");
  DEBUGPRINTNONE("Total: ");
  DEBUGPRINTNONE(EnergyMeter.getExportCounterActivePowerTotal());
  DEBUGPRINTNONE(" L1: ");
  DEBUGPRINTNONE(EnergyMeter.getExportCounterActivePowerL1());
  DEBUGPRINTNONE(" L2: ");
  DEBUGPRINTNONE(EnergyMeter.getExportCounterActivePowerL2());
  DEBUGPRINTNONE(" L3: ");
  DEBUGPRINTLNNONE(EnergyMeter.getExportCounterActivePowerL3());

  DEBUGPRINTLNNONE("Counter Export Reactive Power");
  DEBUGPRINTNONE("Total: ");
  DEBUGPRINTNONE(EnergyMeter.getExportCounterReactivePowerTotal());
  DEBUGPRINTNONE(" L1: ");
  DEBUGPRINTNONE(EnergyMeter.getExportCounterReactivePowerL1());
  DEBUGPRINTNONE(" L2: ");
  DEBUGPRINTNONE(EnergyMeter.getExportCounterReactivePowerL2());
  DEBUGPRINTNONE(" L3: ");
  DEBUGPRINTLNNONE(EnergyMeter.getExportCounterReactivePowerL3());

  DEBUGPRINTNONE("getSerialNo: ");
  DEBUGPRINTLNNONE(EnergyMeter.getSerialNo());
  DEBUGPRINTNONE("getMeterId: ");
  DEBUGPRINTLNNONE(EnergyMeter.getMeterId());
  DEBUGPRINTNONE("getBusBaud: ");
  DEBUGPRINTLNNONE(EnergyMeter.getBusBaud());
  DEBUGPRINTNONE("getSoftwareVersion: ");
  DEBUGPRINTLNNONE(EnergyMeter.getSoftwareVersion());
  DEBUGPRINTNONE("getHardwareVersion: ");
  DEBUGPRINTLNNONE(EnergyMeter.getHardwareVersion());
  DEBUGPRINTNONE("getCountRate: ");
  DEBUGPRINTLNNONE(EnergyMeter.getCountRate());
  DEBUGPRINTNONE("getS0Rate: ");
  DEBUGPRINTLNNONE(EnergyMeter.getS0Rate());
  DEBUGPRINTNONE("getA3: ");
  DEBUGPRINTLNNONE(EnergyMeter.getA3());
  DEBUGPRINTNONE("getCycleTime: ");
  DEBUGPRINTLNNONE(EnergyMeter.getCycleTime());
  DEBUGPRINTNONE("getCrc: ");
  DEBUGPRINTLNNONE(EnergyMeter.getCrc());
  //DEBUGPRINTNONE("getCombinedCode: ");
  //DEBUGPRINTLNNONE(EnergyMeter.getCombinedCode());

  DEBUGPRINTLNNONE("");
  delay(500);
}
