#include <WiFi.h>

extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}

//#include <lblink.h>
#include <AsyncMqttClient.h>

#define WIFI_SSID "TOTO_77"
#define WIFI_PASSWORD "2457100016"

//#define WIFI_SSID "TP-Link_RAK"
//#define WIFI_PASSWORD "00029391"

#define MQTT_HOST IPAddress(34, 118, 69, 241) // VM 001 Warsawa
//MQTT_HOST IPAddress(192, 168, 0, 103) // Local HomeAssistant
//#define MQTT_HOST "mqtt.lanet.io"//"test.mosquitto.org" //"mqtt.lanet.io"
#define MQTT_PORT 1883

#define MQTT_USERNAME "oleh"
#define MQTT_PASSWORD "888"

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
//  uint16_t packetIdSub = mqttClient.subscribe("Keys/test_command", 2);
  uint16_t packetIdSub = mqttClient.subscribe("Oleh_N/SUB", 2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);

  Serial.print("  payload: ");
  Serial.println(payload);  

  Serial.print("  len: ");
  Serial.println(len);  

  Serial.print("  index: ");
  Serial.println(index);

  Serial.print("  total: ");
  Serial.println(total); 

  if (payload == "77") {
    Serial.print(" URA!!! ");
    Serial.println(payload);
  } 

}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);

  connectToWifi();
}
void loop() {
  mqttClient.publish("Oleh_N/PUB/BT_01", 0, true, "ONN");

////uint16_t packetIdPub1 = mqttClient.publish("Oleh_N/lol_2", 1, true, count_MQTT);
  delay(3000);
  mqttClient.publish("Oleh_N/PUB/BT_01", 0, true, "OFF");

  delay(3000);  
}