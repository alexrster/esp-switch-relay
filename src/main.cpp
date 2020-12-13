#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include "version.h"

#ifndef SWITCH_RELAY_PIN
#define SWITCH_RELAY_PIN              0 // GPIO0
#endif

#ifndef MOTION_SENSOR_PIN
#define MOTION_SENSOR_PIN             2 // GPIO2
#endif

#define SWITCH_TIMEOUT_MILLIS         20000
#define MOTION_SENSOR_READ_INTERVAL   100
#define APP_INIT_DELAY_MILLIS         2500

#define WIFI_SSID                     "qx.zone"
#define WIFI_PASSPHRASE               "1234Qwer-"
#define WIFI_RECONNECT_MILLIS         1000
#define WIFI_WATCHDOG_MILLIS          5*60000

#ifndef WIFI_HOSTNAME
#define WIFI_HOSTNAME                 "light-switch-02"
#endif

#define MQTT_SERVER_NAME              "ns2.in.qx.zone"
#define MQTT_SERVER_PORT              1883
#define MQTT_USERNAME                 NULL
#define MQTT_PASSWORD                 NULL
#define MQTT_RECONNECT_MILLIS         5000

#ifndef MQTT_CLIENT_ID
#define MQTT_CLIENT_ID                WIFI_HOSTNAME
#endif

#define MQTT_STATUS_TOPIC             "hallway/light/status"
#define MQTT_MOTION_TOPIC             "hallway/presence"
#define MQTT_SWITCH_STATE_TOPIC       "hallway/light"
#define MQTT_SWITCH_CONTROL_TOPIC     "hallway/light/set"
#define MQTT_RESTART_CONTROL_TOPIC    "hallway/restart"

#define MQTT_STATUS_ONLINE_MSG        "online"
#define MQTT_STATUS_OFFLINE_MSG       "offline"

typedef enum SwitchState : bool { On = true, Off = false } SwitchState_t;
typedef enum MotionState : bool { Detected = true, Idle = false } MotionState_t;

static const String PubSubSwitchControlTopic = String(MQTT_SWITCH_CONTROL_TOPIC);
static const String PubSubRestartControlTopic = String(MQTT_RESTART_CONTROL_TOPIC);

unsigned long 
  now = 0,
  switchOffTime = 0,
  lastWifiOnline = 0,
  lastPubSubReconnectAttempt = 0,
  lastMotionSensorRead = 0;

MotionState_t motionState = Idle;
SwitchState_t switchState = Off;

WiFiClient wifiClient;
PubSubClient pubSubClient(wifiClient);

void onMqttMessage(char* topic, byte* payload, unsigned int length);

void setup() {
  pinMode(MOTION_SENSOR_PIN, INPUT);
  pinMode(SWITCH_RELAY_PIN, OUTPUT);
  digitalWrite(SWITCH_RELAY_PIN, 0);

  WiFi.hostname(WIFI_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);

  ArduinoOTA.setRebootOnSuccess(true);
  ArduinoOTA.begin();

  pubSubClient.setCallback(onMqttMessage);
  pubSubClient.setServer(MQTT_SERVER_NAME, MQTT_SERVER_PORT);
 
  now = millis();
  lastWifiOnline = now;
  lastMotionSensorRead = now + APP_INIT_DELAY_MILLIS; // delay first presence reading
}

bool publishSwitchState() {
  return 
    pubSubClient.connected() && 
    pubSubClient.publish(MQTT_SWITCH_STATE_TOPIC, switchState == On ? "1" : "0", true);
}

bool publishMotionState() {
  return 
    pubSubClient.connected() && 
    pubSubClient.publish(MQTT_MOTION_TOPIC, motionState == Detected ? "1" : "0", true);
}

bool reconnectPubSub() {
  if (now - lastPubSubReconnectAttempt > MQTT_RECONNECT_MILLIS) {
    lastPubSubReconnectAttempt = now;

    if (pubSubClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD, MQTT_STATUS_TOPIC, MQTTQOS0, true, MQTT_STATUS_OFFLINE_MSG, false)) {
      pubSubClient.publish(MQTT_STATUS_TOPIC, VERSION, true);
      
      pubSubClient.subscribe(MQTT_SWITCH_CONTROL_TOPIC, MQTTQOS0);
      pubSubClient.subscribe(MQTT_RESTART_CONTROL_TOPIC, MQTTQOS0);
    }
    
    return pubSubClient.connected();
  }

  return false;
}

void pubSubClientLoop() {
  if (pubSubClient.connected() || reconnectPubSub()) 
    pubSubClient.loop();
}

bool wifiLoop() {
  if (WiFi.status() != WL_CONNECTED) {
    if(now - lastWifiOnline > WIFI_WATCHDOG_MILLIS) ESP.restart();

    return false;
  }
  
  lastWifiOnline = now;
  return true;
}

void setSwitch(SwitchState_t newSwitchState) {
  if (switchState == On) switchOffTime = now + SWITCH_TIMEOUT_MILLIS;
  if (switchState == newSwitchState) return;
 
  switchState = newSwitchState;
  digitalWrite(SWITCH_RELAY_PIN, switchState == On ? 1 : 0);
  
  publishSwitchState();
}

void loop() {
  now = millis();

  if (wifiLoop()) {
    pubSubClientLoop();
    ArduinoOTA.handle();
  }

  if (now - lastMotionSensorRead > MOTION_SENSOR_READ_INTERVAL) {
    lastMotionSensorRead = now;

    MotionState_t newMotionState = digitalRead(MOTION_SENSOR_PIN) > 0 ? Detected : Idle;
    if (motionState != newMotionState) {
      motionState = newMotionState;
      publishMotionState();
    }

    if (motionState == Detected) 
      setSwitch(On);
  }

  if (switchState == On && now >= switchOffTime) 
    setSwitch(Off);
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  if (!length) return;

  String msg = String((char*)payload);
  msg.setCharAt(length, 0);
  msg.trim();
  msg.toLowerCase();

  if (PubSubSwitchControlTopic == (String)topic) {
    if (msg.equals("1") || msg.equals("on") || msg.equals("true")) setSwitch(On);
    else if (msg.equals("0") || msg.equals("off") || msg.equals("false")) setSwitch(Off);
  }
  else if (PubSubRestartControlTopic == (String)topic) {
    if (msg == "1") ESP.restart();
    else if (msg == "2") ESP.reset();
  }
}
