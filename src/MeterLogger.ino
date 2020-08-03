/*
   Program for isolating and publishing smartmeter modbus data to a mqtt broker by using a esp32.
*/

#define LED_PIN 2
#define ENERGY_RX_PIN 4  // for NodeMCU: GPIO4 = D1
#define ENERGY_TX_PIN 15 // for NodeMCU: GPIO5 = D2
#define ENERGY_ADR 1     // Modbus adress
#define DEBUGLEVEL DEBUG
#define MAX_PACKET_SIZE 512 // Max data packet size

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 10       /* Time ESP32 will go to sleep (in seconds) */

#include <WiFi.h>
#include "OTA.h"
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_esp8266/mqtt_esp8266.ino
#include <HardwareSerial.h>
#include <ModbusMaster.h>
#include <DebugUtils.h>
#include <ArduinoJson.h>
#include <OR_WE.h>

//#include "Config.h" // make your own config file or remove this line and use the following lines
const char *clientId = "Energy";
const char *mqtt_server = "192.168.2.64";
#include "WifiCredentials.h" // const char* ssid = "MySSID"; const char* WifiPassword = "MyPw";
IPAddress ip(192, 168, 2, 7);      // Static IP
IPAddress dns(192, 168, 2, 1);     // most likely your router
IPAddress gateway(192, 168, 2, 1); // most likely your router
IPAddress subnet(255, 255, 255, 0);

unsigned long lastUpdated;
unsigned long lastLed;
unsigned long entry;
const char *nameprefix = "MeterLogger";
uint8_t stateLed = HIGH;
boolean updateActive;
RTC_DATA_ATTR unsigned long bootCount;
RTC_DATA_ATTR boolean enableUpdate;

OR_WE EnergyMeter;
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
  const int maxlen = 40;
  char fullhostname[maxlen];
  uint8_t mac[6];
  delay(10);
  DEBUGPRINTNONE("Connecting to ");
  DEBUGPRINTLNNONE(ssid);

  WiFi.macAddress(mac);
  snprintf(fullhostname, maxlen, "%s-%02x%02x%02x", nameprefix, mac[3], mac[4], mac[5]);
  WiFi.setHostname(fullhostname);
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

  if (enableUpdate)
  {
    setupOTA(fullhostname);
  }
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

void setup()
{
  bootCount++;
  updateActive = enableUpdate;
  lastUpdated = millis();
  lastLed = millis();
  Serial.begin(115200);
  DEBUGPRINTLNNONE("\nHardware serial started");
  DEBUGPRINTLNNONE(bootCount);
  pinMode(LED_PIN, OUTPUT);
  if (!enableUpdate)
  {
    Serial1.begin(OR_WE_SERIAL_BAUD, OR_WE_SERIAL_MODE, ENERGY_RX_PIN, ENERGY_TX_PIN);
    // communicate with Modbus slave ID 1 over Serial (port 1)
    EnergyMeter.begin(Serial1, ENERGY_ADR);
    DEBUGPRINTLNNONE("\nEnergy meter serial started");
  }
  setup_wifi();
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(MAX_PACKET_SIZE);
}

void loop()
{
  if (enableUpdate && !updateActive)
  {
    esp_deep_sleep(10 * uS_TO_S_FACTOR);
  }
  else if (enableUpdate)
  {
    ArduinoOTA.handle();
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    char Data[MAX_PACKET_SIZE];

    if (!mqttClient.connected())
    {
      mqttReconnect();
    }
    mqttClient.loop();

    if (millis() - lastUpdated >= 10000)
    {
      float value;
      const size_t capacity = JSON_OBJECT_SIZE(9) + JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(4) + 33 * sizeof(float);
      DynamicJsonDocument doc(capacity);

      // Voltage
      JsonObject Voltage = doc.createNestedObject("Voltage");
      DEBUGPRINTLNDEBUG("Voltage");
      value = EnergyMeter.getVoltageL1();
      Voltage["L1"] = value;
      DEBUGPRINTDEBUG("L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getVoltageL2();
      Voltage["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getVoltageL3();
      Voltage["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTLNDEBUG(value);

      JsonObject Freq = doc.createNestedObject("Freq");
      value = EnergyMeter.getFrequency();
      Freq["value"] = value;
      DEBUGPRINTDEBUG("Freq: ");
      DEBUGPRINTLNDEBUG(value);

      //Current
      JsonObject Current = doc.createNestedObject("Current");
      DEBUGPRINTLNDEBUG("Current");
      value = EnergyMeter.getCurrentL1();
      Current["L1"] = value;
      DEBUGPRINTDEBUG("L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getCurrentL2();
      Current["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getCurrentL3();
      Current["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTLNDEBUG(value);

      //Active Power
      JsonObject ActPower = doc.createNestedObject("ActivePower");
      value = EnergyMeter.getActivePowerTotal();
      DEBUGPRINTNONE("Bezug: ");
      DEBUGPRINTNONE(value, 0);
      DEBUGPRINTLNNONE("W");
      DEBUGPRINTLNDEBUG("ActivePower");
      ActPower["Total"] = value;
      DEBUGPRINTDEBUG("Total ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getActivePowerL1();
      ActPower["L1"] = value;
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getActivePowerL2();
      ActPower["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getActivePowerL3();
      ActPower["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTLNDEBUG(value);

      //Reactive Power
      JsonObject ReactPower = doc.createNestedObject("ReactivePower");
      DEBUGPRINTLNDEBUG("Reactive Power");
      value = EnergyMeter.getReactivePowerTotal();
      ReactPower["Total"] = value;
      DEBUGPRINTDEBUG("Total: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getReactivePowerL1();
      ReactPower["L1"] = value;
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getReactivePowerL2();
      ReactPower["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getReactivePowerL3();
      ReactPower["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTLNDEBUG(value);

      //Apparent Power
      JsonObject AppPower = doc.createNestedObject("ApparentPower");
      DEBUGPRINTLNDEBUG("Apparent Power");
      value = EnergyMeter.getApparentPowerTotal();
      AppPower["Total"] = value;
      DEBUGPRINTDEBUG("Total: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getApparentPowerL1();
      AppPower["L1"] = value;
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getApparentPowerL2();
      AppPower["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getApparentPowerL3();
      AppPower["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTLNDEBUG(value);

      //Power Factor
      JsonObject PowerFactor = doc.createNestedObject("PowerFactor");
      DEBUGPRINTLNDEBUG("Power Factor");
      value = EnergyMeter.getPowerFactorTotal();
      PowerFactor["Total"] = value;
      DEBUGPRINTDEBUG("Total: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getPowerFactorL1();
      PowerFactor["L1"] = value;
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getPowerFactorL2();
      PowerFactor["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getPowerFactorL3();
      PowerFactor["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTLNDEBUG(value);

      //Counter
      JsonObject CounterActPower = doc.createNestedObject("CountActPower");
      value = EnergyMeter.getTotalCounterActivePowerTotal();
      DEBUGPRINTNONE("Verbrauch: ");
      DEBUGPRINTNONE(value);
      DEBUGPRINTLNNONE("kWh ");
      DEBUGPRINTLNDEBUG("Counter Active Power");
      CounterActPower["Total"] = value;
      DEBUGPRINTDEBUG("Total: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getTotalCounterActivePowerL1();
      CounterActPower["L1"] = value;
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getTotalCounterActivePowerL2();
      CounterActPower["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getTotalCounterActivePowerL3();
      CounterActPower["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTLNDEBUG(value);

      JsonObject CounterReactPower = doc.createNestedObject("CountReactPower");
      DEBUGPRINTLNDEBUG("Counter Reactive Power");
      value = EnergyMeter.getTotalCounterReactivePowerTotal();
      CounterReactPower["Total"] = value;
      DEBUGPRINTDEBUG("Total: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getTotalCounterReactivePowerL1();
      CounterReactPower["L1"] = value;
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getTotalCounterReactivePowerL2();
      CounterReactPower["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.getTotalCounterReactivePowerL3();
      CounterReactPower["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTLNDEBUG(value);

      //Counter Import
      DEBUGPRINTLNDEBUG("Counter Import Active Power");
      DEBUGPRINTDEBUG("Total: ");
      DEBUGPRINTDEBUG(EnergyMeter.getImportCounterActivePowerTotal());
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(EnergyMeter.getImportCounterActivePowerL1());
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(EnergyMeter.getImportCounterActivePowerL2());
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTLNDEBUG(EnergyMeter.getImportCounterActivePowerL3());

      DEBUGPRINTLNDEBUG("Counter Import Reactive Power");
      DEBUGPRINTDEBUG("Total: ");
      DEBUGPRINTDEBUG(EnergyMeter.getImportCounterReactivePowerTotal());
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(EnergyMeter.getImportCounterReactivePowerL1());
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(EnergyMeter.getImportCounterReactivePowerL2());
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTLNDEBUG(EnergyMeter.getImportCounterReactivePowerL3());

      //Counter Export
      DEBUGPRINTLNDEBUG("Counter Export Active Power");
      DEBUGPRINTDEBUG("Total: ");
      DEBUGPRINTDEBUG(EnergyMeter.getExportCounterActivePowerTotal());
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(EnergyMeter.getExportCounterActivePowerL1());
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(EnergyMeter.getExportCounterActivePowerL2());
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTLNDEBUG(EnergyMeter.getExportCounterActivePowerL3());

      DEBUGPRINTLNDEBUG("Counter Export Reactive Power");
      DEBUGPRINTDEBUG("Total: ");
      DEBUGPRINTDEBUG(EnergyMeter.getExportCounterReactivePowerTotal());
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(EnergyMeter.getExportCounterReactivePowerL1());
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(EnergyMeter.getExportCounterReactivePowerL2());
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTLNDEBUG(EnergyMeter.getExportCounterReactivePowerL3());

      DEBUGPRINTDEBUG("getSerialNo: ");
      DEBUGPRINTLNDEBUG(EnergyMeter.getSerialNo());
      DEBUGPRINTDEBUG("getMeterId: ");
      DEBUGPRINTLNDEBUG(EnergyMeter.getMeterId());
      DEBUGPRINTDEBUG("getBusBaud: ");
      DEBUGPRINTLNDEBUG(EnergyMeter.getBusBaud());
      DEBUGPRINTDEBUG("getSoftwareVersion: ");
      DEBUGPRINTLNDEBUG(EnergyMeter.getSoftwareVersion());
      DEBUGPRINTDEBUG("getHardwareVersion: ");
      DEBUGPRINTLNDEBUG(EnergyMeter.getHardwareVersion());
      DEBUGPRINTDEBUG("getCountRate: ");
      DEBUGPRINTLNDEBUG(EnergyMeter.getCountRate());
      DEBUGPRINTDEBUG("getS0Rate: ");
      DEBUGPRINTLNDEBUG(EnergyMeter.getS0Rate());
      DEBUGPRINTDEBUG("getA3: ");
      DEBUGPRINTLNDEBUG(EnergyMeter.getA3());
      DEBUGPRINTDEBUG("getCycleTime: ");
      DEBUGPRINTLNDEBUG(EnergyMeter.getCycleTime());
      DEBUGPRINTDEBUG("getCrc: ");
      DEBUGPRINTLNDEBUG(EnergyMeter.getCrc());

      DEBUGPRINTDEBUG("MemUsage.........: ");
      DEBUGPRINTLNDEBUG(doc.memoryUsage());

      serializeJson(doc, Data, sizeof(Data));
      char *topic = "/MeterData";
      char *path = (char *)malloc(1 + strlen(clientId) + strlen(topic));
      strcpy(path, clientId);
      strcat(path, topic);

      DEBUGPRINTDEBUG("StringSize.........: ");
      DEBUGPRINTLNDEBUG(strlen(Data));

      if (mqttClient.publish(path, Data, true))
      {
        lastUpdated = millis();
      }
      else
      {
        DEBUGPRINTLNNONE("MQTT publish failed");
      }
      free(path);

      DEBUGPRINTDEBUG(topic);
      DEBUGPRINTDEBUG(" ");
      DEBUGPRINTLNDEBUG(Data);

      delay(100);
      update_led(500, 500);

      lastUpdated = millis();
    }
    else if (millis() - lastUpdated < 500)
    {
      update_led(500, 500);
    }
    else
    {
      update_led(100, 1000);
    }
  }
}
