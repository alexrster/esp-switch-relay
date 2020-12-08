#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266httpUpdate.h>
#include <ArduinoOTA.h>
#include "version.h"

#ifndef SWITCH_RELAY_PIN
#define SWITCH_RELAY_PIN              0 // GPIO0
#endif

#ifndef PRESENCE_SENSOR_PIN
#define PRESENCE_SENSOR_PIN           2 // GPIO2
#endif

#define RELAY_SWITCH_DELAY            100
#define RELAY_SWITCH_TIMEOUT          600000 // 10 minutes
#define PRESENCE_SENSOR_DELAY         100
#define APP_INIT_TIMEOUT              2500 // 3 seconds

#define WIFI_SSID                     "qx.zone"
#define WIFI_PASSPHRASE               "1234Qwer-"
#define WIFI_RECONNECT_MILLIS         5000

#ifndef MQTT_CLIENT_ID
#define MQTT_CLIENT_ID                "light-switch"
#endif

#define MQTT_CONTROL_TOPIC            "hallway/light/set"
#define MQTT_STATE_TOPIC              "hallway/light"
#define MQTT_STATUS_TOPIC             "hallway/light/status"
#define MQTT_PRESENCE_TOPIC           "hallway/presence"
#define MQTT_SETTINGS_TOPIC           "hallway/settings"
#define MQTT_OTA_UPDATE_TOPIC         "hallway/ota/update"
#define MQTT_OTA_STATUS_TOPIC         "hallway/ota/status"
#define MQTT_RESTART_TOPIC            "hallway/restart"

#define MQTT_AT_MOST_ONCE             0
#define MQTT_AT_LEAST_ONCE            1

#define CHECK_FLAG(x, f) (!((x &~ f) == x))

enum AppSettings: uint8_t {
  PowerOnState = (1 << 0),
  PresenceAutoSwitchOn = (1 << 1),
  PresenceAutoSwitchOff = (1 << 2)
  // Last 4 bits reserved to timeout minutes value
};

void onMqttMessage(char* topic, byte* payload, unsigned int length);

const byte mqtt_server[] = { 10, 9, 9, 96 };

unsigned long 
  lastWifiReconnect = 0,
  lastRelaySwitch = 0,
  lastPresenceRead = 0,
  switchTimeoutMillis = RELAY_SWITCH_TIMEOUT,
  appInitMillis = 0;

uint8_t
  appSettings = 0b10100110, // 10 minutes timeout
  appInitComplete = 0,
  currentState = 0,
  targetState = 0,
  presenceState = 0;

WiFiClient wifiClient;
PubSubClient mqttClient = PubSubClient(mqtt_server, 1883, onMqttMessage, wifiClient);

void publishCurrentState() {
  mqttClient.publish(MQTT_STATE_TOPIC, currentState ? "1" : "0", true);
}

void publishCurrentPresence() {
  mqttClient.publish(MQTT_PRESENCE_TOPIC, presenceState ? "1" : "0", true);
}

bool checkMqtt() {
  if (mqttClient.connected()) return true;

  if (mqttClient.connect(MQTT_CLIENT_ID, NULL, NULL, MQTT_STATUS_TOPIC, MQTT_AT_LEAST_ONCE, true, "offline", false)) {
    mqttClient.publish(MQTT_STATUS_TOPIC, "online", true);
    mqttClient.subscribe(MQTT_CONTROL_TOPIC, MQTT_AT_MOST_ONCE);
    mqttClient.subscribe(MQTT_SETTINGS_TOPIC, MQTT_AT_LEAST_ONCE);
    mqttClient.subscribe(MQTT_OTA_UPDATE_TOPIC, MQTT_AT_MOST_ONCE);
    mqttClient.subscribe(MQTT_RESTART_TOPIC, MQTT_AT_MOST_ONCE);

    publishCurrentState();
    publishCurrentPresence();

    return true;
  }

  return false;
}

void setup() {
  #ifdef DEBUG
  Serial.begin(9600);
  #endif

  pinMode(SWITCH_RELAY_PIN, OUTPUT);
  digitalWrite(SWITCH_RELAY_PIN, currentState);

  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);

  ArduinoOTA.begin();

  appInitMillis = lastPresenceRead = millis(); // delay first presence reading
}

void loop() {
  if ((!WiFi.isConnected() || !checkMqtt()) && millis() - lastWifiReconnect > WIFI_RECONNECT_MILLIS) {
    lastWifiReconnect = millis();
    WiFi.reconnect();

    return;
  }

  #ifdef DEBUG
  Serial.print(WiFi.localIP());
  #endif

  if (!appInitComplete) {
    if (millis() - appInitMillis > APP_INIT_TIMEOUT) {
      appInitComplete = 1;
      targetState = CHECK_FLAG(appSettings, AppSettings::PowerOnState) ? 1 : 0;
    }
  }

  if (millis() - lastPresenceRead > PRESENCE_SENSOR_DELAY) {
    lastPresenceRead = millis();

    bool presenceDetected = (digitalRead(PRESENCE_SENSOR_PIN) != 0);
    if (presenceDetected != (presenceState > 0)) {
      presenceState = presenceDetected ? 1 : 0;

      publishCurrentPresence();

      if ((presenceState && CHECK_FLAG(appSettings, AppSettings::PresenceAutoSwitchOn)) ||
        (!presenceState && CHECK_FLAG(appSettings, AppSettings::PresenceAutoSwitchOff))) {
        targetState = presenceState;
      }
    }
  }

  if (currentState && millis() - lastRelaySwitch > switchTimeoutMillis) {
    targetState = 0;
  }

  if (currentState != targetState && millis() - lastRelaySwitch > RELAY_SWITCH_DELAY) {
    lastRelaySwitch = millis();

    digitalWrite(SWITCH_RELAY_PIN, targetState);
    currentState = targetState;
    
    publishCurrentState();
  }

  mqttClient.loop();
  ArduinoOTA.handle();
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  #ifdef DEBUG
  Serial.print("MQTT message handler for: ");
  Serial.println(topic);
  Serial.println((char*)payload);
  Serial.println();
  #endif

  String topicStr = String(topic);
  if (topicStr == MQTT_CONTROL_TOPIC) {
    if (length == 0) return;

    String msg = String((char*)payload);
    msg.setCharAt(length, 0);
    msg.trim();
    msg.toLowerCase();

    if ((msg == "on" || msg == "1" || msg == "true") && msg != "0") targetState = 1;
    else targetState = 0;

    #ifdef DEBUG
    Serial.print("Updated target state to: ");
    Serial.println(targetState);
    #endif
  }
  else if (topicStr == MQTT_SETTINGS_TOPIC) {
    if (length != 1) return;
    appSettings = payload[0];
    switchTimeoutMillis = (appSettings >> 4) * 60000;
    if (switchTimeoutMillis == 0) switchTimeoutMillis = RELAY_SWITCH_TIMEOUT;
    appInitMillis = 0;
  }
  else if (topicStr == MQTT_OTA_UPDATE_TOPIC) {
    if (length == 0) return;

    String msg = String((char*)payload);
    msg.setCharAt(length, 0);

    char statusStr[255];
    t_httpUpdate_return ret = ESPhttpUpdate.update(wifiClient, msg);
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        sprintf(statusStr, "HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;
      case HTTP_UPDATE_NO_UPDATES:
        sprintf(statusStr, "HTTP_UPDATE_NO_UPDATES");
        break;
      case HTTP_UPDATE_OK:
        sprintf(statusStr, "HTTP_UPDATE_OK");
        break;
    }

    mqttClient.publish(MQTT_OTA_STATUS_TOPIC, statusStr, true);
    ESP.restart();
  }
  else if (topicStr == MQTT_RESTART_TOPIC) {
    ESP.restart();
  }
}
