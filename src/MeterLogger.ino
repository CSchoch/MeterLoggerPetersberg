/*
   Program for isolating and publishing smartmeter modbus data to a mqtt broker by using a esp32.
*/

#define LED_PIN 2
#define ENERGY_RX_PIN 4  // for NodeMCU: GPIO4 = D1
#define ENERGY_TX_PIN 15 // for NodeMCU: GPIO5 = D2
#define ENERGY_ADR 1     // Modbus adress
#define DEBUGLEVEL NONE
#define MAX_PACKET_SIZE 1024 // Max data packet size

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 10       /* Time ESP32 will go to sleep (in seconds) */

#include <WiFi.h>
#include "OTA.h"
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_esp8266/mqtt_esp8266.ino
#include <HardwareSerial.h>
#include <DebugUtils.h>
#include <ArduinoJson.h>
#include <SDM.h>

//#include "Config.h" // make your own config file or remove this line and use the following lines
const char *clientId = "Energy2";
const char *mqtt_server = "192.168.2.64";
#include "WifiCredentials.h"       // const char* ssid = "MySSID"; const char* WifiPassword = "MyPw";
IPAddress ip(192, 168, 2, 10);      // Static IP
IPAddress dns(192, 168, 2, 1);     // most likely your router
IPAddress gateway(192, 168, 2, 1); // most likely your router
IPAddress subnet(255, 255, 255, 0);

unsigned long lastUpdated;
unsigned long lastLed;
unsigned long entry;
const char *nameprefix = "MeterLogger2";
uint8_t stateLed = HIGH;
boolean updateActive;
RTC_DATA_ATTR unsigned long bootCount;
RTC_DATA_ATTR boolean enableUpdate;
size_t DebugMessage;

SDM EnergyMeter(Serial1, 9600, NOT_A_PIN, SDM_UART_CONFIG, ENERGY_RX_PIN, ENERGY_TX_PIN);
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

void setup()
{
  Serial.begin(115200);
  DEBUGPRINTLNNONE("\nHardware serial started");
  bootCount++;
  updateActive = enableUpdate;
  lastUpdated = millis();
  lastLed = millis();
  DEBUGPRINTLNNONE(bootCount);
  pinMode(LED_PIN, OUTPUT);
  if (!enableUpdate)
  {
    //Serial1.begin(OR_WE_SERIAL_BAUD, OR_WE_SERIAL_MODE, ENERGY_RX_PIN, ENERGY_TX_PIN);
    // communicate with Modbus slave ID 1 over Serial (port 1)
    //EnergyMeter.begin(Serial1, ENERGY_ADR);
    EnergyMeter.begin();
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

    if (millis() - lastUpdated >= TIME_TO_SLEEP * 1000)
    {
      lastUpdated = millis();
      float value;
      DynamicJsonDocument doc(1024);

      // Voltage
  
      JsonObject Voltage = doc.createNestedObject("Voltage");
      DEBUGPRINTLNDEBUG("Voltage");
      value = EnergyMeter.readVal(SDM_PHASE_1_VOLTAGE);
      Voltage["L1"] = value;
      DEBUGPRINTDEBUG("L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_PHASE_2_VOLTAGE);
      Voltage["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_PHASE_3_VOLTAGE);
      Voltage["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTDEBUG(value);
      DEBUGPRINTLNDEBUG(" V");

      JsonObject Freq = doc.createNestedObject("Freq");
      value = EnergyMeter.readVal(SDM_FREQUENCY);
      Freq["value"] = value;
      DEBUGPRINTDEBUG("Freq: ");
      DEBUGPRINTDEBUG(value);
      DEBUGPRINTLNDEBUG(" Hz");

      //Current
      JsonObject Current = doc.createNestedObject("Current");
      DEBUGPRINTLNDEBUG("Current");
      value = EnergyMeter.readVal(SDM_PHASE_1_CURRENT);
      Current["L1"] = value;
      DEBUGPRINTDEBUG("L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_PHASE_2_CURRENT);
      Current["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_PHASE_3_CURRENT);
      Current["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTDEBUG(value);
      DEBUGPRINTLNDEBUG(" A");

      //Active Power
      JsonObject ActPower = doc.createNestedObject("ActivePower");
      value = EnergyMeter.readVal(SDM_TOTAL_SYSTEM_POWER);
      DEBUGPRINTNONE("Bezug: ");
      DEBUGPRINTNONE(value, 0);
      DEBUGPRINTLNNONE("W");
      DEBUGPRINTLNDEBUG("ActivePower");
      ActPower["Total"] = value;
      DEBUGPRINTDEBUG("Total ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_PHASE_1_POWER);
      ActPower["L1"] = value;
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_PHASE_2_POWER);
      ActPower["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_PHASE_3_POWER);
      ActPower["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTDEBUG(value);
      DEBUGPRINTLNDEBUG(" W");

      //Reactive Power
      JsonObject ReactPower = doc.createNestedObject("ReactivePower");
      DEBUGPRINTLNDEBUG("Reactive Power");
      value = EnergyMeter.readVal(SDM_TOTAL_SYSTEM_REACTIVE_POWER);
      ReactPower["Total"] = value;
      DEBUGPRINTDEBUG("Total: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_PHASE_1_REACTIVE_POWER);
      ReactPower["L1"] = value;
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_PHASE_2_REACTIVE_POWER);
      ReactPower["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_PHASE_3_REACTIVE_POWER);
      ReactPower["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTDEBUG(value);
      DEBUGPRINTLNDEBUG(" var");

      //Apparent Power
      JsonObject AppPower = doc.createNestedObject("ApparentPower");
      DEBUGPRINTLNDEBUG("Apparent Power");
      value = EnergyMeter.readVal(SDM_TOTAL_SYSTEM_APPARENT_POWER);
      AppPower["Total"] = value;
      DEBUGPRINTDEBUG("Total: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_PHASE_1_APPARENT_POWER);
      AppPower["L1"] = value;
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_PHASE_2_APPARENT_POWER);
      AppPower["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_PHASE_3_APPARENT_POWER);
      AppPower["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTDEBUG(value);
      DEBUGPRINTLNDEBUG(" VA");

      //Power Factor
      JsonObject PowerFactor = doc.createNestedObject("PowerFactor");
      DEBUGPRINTLNDEBUG("Power Factor");
      value = EnergyMeter.readVal(SDM_TOTAL_SYSTEM_POWER_FACTOR);
      PowerFactor["Total"] = value;
      DEBUGPRINTDEBUG("Total: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_PHASE_1_POWER_FACTOR);
      PowerFactor["L1"] = value;
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_PHASE_2_POWER_FACTOR);
      PowerFactor["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_PHASE_3_POWER_FACTOR);
      PowerFactor["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTLNDEBUG(value);

      //Counter
      JsonObject CounterActPower = doc.createNestedObject("CountActPower");
      value = EnergyMeter.readVal(SDM_TOTAL_ACTIVE_ENERGY);
      DEBUGPRINTNONE("Verbrauch: ");
      DEBUGPRINTNONE(value);
      DEBUGPRINTLNNONE("kWh ");
      DEBUGPRINTLNDEBUG("Counter Active Power");
      CounterActPower["Total"] = value;
      DEBUGPRINTDEBUG("Total: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_L1_TOTAL_ACTIVE_ENERGY);
      CounterActPower["L1"] = value;
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_L2_TOTAL_ACTIVE_ENERGY);
      CounterActPower["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_L3_TOTAL_ACTIVE_ENERGY);
      CounterActPower["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTDEBUG(value);
      DEBUGPRINTLNDEBUG(" kWh");

      JsonObject CounterReactPower = doc.createNestedObject("CountReactPower");
      DEBUGPRINTLNDEBUG("Counter Reactive Power");
      value = EnergyMeter.readVal(SDM_TOTAL_REACTIVE_ENERGY);
      CounterReactPower["Total"] = value;
      DEBUGPRINTDEBUG("Total: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_L1_TOTAL_REACTIVE_ENERGY);
      CounterReactPower["L1"] = value;
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_L2_TOTAL_REACTIVE_ENERGY);
      CounterReactPower["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_L3_TOTAL_REACTIVE_ENERGY);
      CounterReactPower["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTDEBUG(value);
      DEBUGPRINTLNDEBUG(" kvarh");
      //Counter Import
      JsonObject CounterImportActPower = doc.createNestedObject("CountImportActPower");
      DEBUGPRINTLNDEBUG("Counter Import Active Power");
      value = EnergyMeter.readVal(SDM_IMPORT_ACTIVE_ENERGY);
      CounterImportActPower["Total"] = value;
      DEBUGPRINTDEBUG("Total: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_L1_IMPORT_ACTIVE_ENERGY);
      CounterImportActPower["L1"] = value;
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_L2_IMPORT_ACTIVE_ENERGY);
      CounterImportActPower["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_L3_IMPORT_ACTIVE_ENERGY);
      CounterImportActPower["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTDEBUG(value);
      DEBUGPRINTLNDEBUG(" kWh");

      JsonObject CounterImportReactPower = doc.createNestedObject("CountImportReactPower");
      DEBUGPRINTLNDEBUG("Counter Import Reactive Power");
      value = EnergyMeter.readVal(SDM_IMPORT_REACTIVE_ENERGY);
      CounterImportReactPower["Total"] = value;
      DEBUGPRINTDEBUG("Total: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_L1_IMPORT_REACTIVE_ENERGY);
      CounterImportReactPower["L1"] = value;
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_L2_IMPORT_REACTIVE_ENERGY);
      CounterImportReactPower["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_L3_IMPORT_REACTIVE_ENERGY);
      CounterImportReactPower["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTDEBUG(value);
      DEBUGPRINTLNDEBUG(" kvarh");

      //Counter Export
      JsonObject CounterExportActPower = doc.createNestedObject("CountExportActPower");
      DEBUGPRINTLNDEBUG("Counter Export Active Power");
      value = EnergyMeter.readVal(SDM_EXPORT_ACTIVE_ENERGY);
      CounterExportActPower["Total"] = value;
      DEBUGPRINTDEBUG("Total: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_L1_EXPORT_ACTIVE_ENERGY);
      CounterExportActPower["L1"] = value;
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_L2_EXPORT_ACTIVE_ENERGY);
      CounterExportActPower["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_L3_EXPORT_ACTIVE_ENERGY);
      CounterExportActPower["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTDEBUG(value);
      DEBUGPRINTLNDEBUG(" kWh");

      JsonObject CounterExportReactPower = doc.createNestedObject("CountExportReactPower");
      DEBUGPRINTLNDEBUG("Counter Export Reactive Power");
      value = EnergyMeter.readVal(SDM_EXPORT_REACTIVE_ENERGY);
      CounterExportReactPower["Total"] = value;
      DEBUGPRINTDEBUG("Total: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_L1_EXPORT_REACTIVE_ENERGY);
      CounterExportReactPower["L1"] = value;
      DEBUGPRINTDEBUG(" L1: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_L2_EXPORT_REACTIVE_ENERGY);
      CounterExportReactPower["L2"] = value;
      DEBUGPRINTDEBUG(" L2: ");
      DEBUGPRINTDEBUG(value);
      value = EnergyMeter.readVal(SDM_L3_EXPORT_REACTIVE_ENERGY);
      CounterExportReactPower["L3"] = value;
      DEBUGPRINTDEBUG(" L3: ");
      DEBUGPRINTDEBUG(value);
      DEBUGPRINTLNDEBUG(" kvarh");

      // DEBUGPRINTDEBUG("getSerialNo: ");
      // DEBUGPRINTLNDEBUG(EnergyMeter.getSerialNo());
      // DEBUGPRINTDEBUG("getMeterId: ");
      // DEBUGPRINTLNDEBUG(EnergyMeter.getMeterId());
      // DEBUGPRINTDEBUG("getBusBaud: ");
      // DEBUGPRINTLNDEBUG(EnergyMeter.getBusBaud());
      // DEBUGPRINTDEBUG("getSoftwareVersion: ");
      // DEBUGPRINTLNDEBUG(EnergyMeter.getSoftwareVersion());
      // DEBUGPRINTDEBUG("getHardwareVersion: ");
      // DEBUGPRINTLNDEBUG(EnergyMeter.getHardwareVersion());
      // DEBUGPRINTDEBUG("getCountRate: ");
      // DEBUGPRINTLNDEBUG(EnergyMeter.getCountRate());
      // DEBUGPRINTDEBUG("getS0Rate: ");
      // DEBUGPRINTLNDEBUG(EnergyMeter.getS0Rate());
      // DEBUGPRINTDEBUG("getA3: ");
      // DEBUGPRINTLNDEBUG(EnergyMeter.getA3());
      // DEBUGPRINTDEBUG("getCycleTime: ");
      // DEBUGPRINTLNDEBUG(EnergyMeter.getCycleTime());
      // DEBUGPRINTDEBUG("getCrc: ");
      // DEBUGPRINTLNDEBUG(EnergyMeter.getCrc());

      DEBUGPRINTDEBUG("MemUsage.........: ");
      DEBUGPRINTLNDEBUG(doc.memoryUsage());

      serializeJson(doc, Data, sizeof(Data));
      char *topic = "/MeterData";
      char *path = (char *)malloc(1 + strlen(clientId) + strlen(topic));
      strcpy(path, clientId);
      strcat(path, topic);

      DEBUGPRINTDEBUG("StringSize.........: ");
      DebugMessage = strlen(Data);
      DEBUGPRINTLNDEBUG(strlen(Data));

      if (!mqttClient.publish(path, Data, false))
      {
        lastUpdated = millis() - 10000;
        DEBUGPRINTLNNONE("MQTT publish failed");
      }
      free(path);

      DEBUGPRINTDEBUG(topic);
      DEBUGPRINTDEBUG(" ");
      DEBUGPRINTLNDEBUG(Data);

      delay(100);
      update_led(500, 500);
    }
    else
    {
      update_led(100, 1000);
    }
  }
}
