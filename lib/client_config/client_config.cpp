#include "client_config.h"
#include "sensor_config.h"
#include <WiFi.h>
#include <PubSubClient.h>

// MQTT Broker details
const char *mqttServer = "112.137.129.171";
const int mqttPort = 1883;
const char *mqttUser = "";
const char *mqttPassword = "";

String MQTT_topic[6] = {"command/1", "command/2", "command/3",
                        "command/4", "command/5", "command/6"};

// Variables to store the current state of each device (ON/OFF)
String DeviceState[6] = {"0", "0", "0", "0", "0", "0"};

// Assign each device to a GPIO pin
typedef enum
{
  device_1 = 0,
  device_2,
  device_3,
  device_4,
  device_5,
  device_6
} device_x;
uint8_t Device_pin[6] = {25, 26, 27, 14, 32, 33};

bool autoMode = true; // 1-auto 0-manual

// Initialize the WiFi and MQTT client objects
WiFiClient espClient;
PubSubClient client(espClient);

void client_reconnect(void)
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.println("Attempting MQTT connection...");
    // Attemp to connect
    if (client.connect("ESP32Client", mqttUser, mqttPassword))
    {
      Serial.println("Connected");
      // Once connected, publish an announcement...
      client_publish_status();
      // Subscribe to topics
      client_subscribe();
    }
    else
    {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
  
}

void callback(char *topic, byte *message, unsigned int length)
{
  String messageTemp;

  for (int i = 0; i < length; i++)
  {
    messageTemp += (char)message[i];
  }

  // Print the message for debugging
  // Serial.print("Message arrived on topic: ");
  // Serial.print(topic);
  // Serial.print(". Message: ");
  // Serial.println(messageTemp);

  // Check the received message and update the corresponding relay state
  String topicStr = String(topic);

  if (topicStr == MQTT_topic[device_1])
  {
    handle_message(messageTemp, device_1);
  }
  else if (topicStr == MQTT_topic[device_2])
  {
    handle_message(messageTemp, device_2);
  }
  else if (topicStr == MQTT_topic[device_3])
  {
    handle_message(messageTemp, device_3);
  }
  else if (topicStr == MQTT_topic[device_4])
  {
    handle_message(messageTemp, device_4);
  }
  else if (topicStr == MQTT_topic[device_5])
  {
    handle_message(messageTemp, device_5);
  }
  else if (topicStr == MQTT_topic[device_6])
  {
    handle_message(messageTemp, device_6);
  }
  if(strcmp(topic,"change-mode/1/") == 0){
    if ((char)message[0] == '1') {
      TurnOnAutoMode();
    } else {
      TurnOffAutoMode();
    }
    return;
  }
}

void init_client(void)
{
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
}

bool check_client_connect()
{
  return client.connected();
}

void handle_message(String message, int device_index)
{
  if (message == "1")
  {
    Serial.printf("Device %d on", device_index + 1);
    DeviceState[device_index] = "1";
    digitalWrite(Device_pin[device_index], LOW);
    String topic = "device-status/" + String(device_index + 1) + "/";
    client.publish(topic.c_str(), "1");
  }
  else if (message == "0")
  {
    Serial.printf("Device %d off", device_index + 1);
    DeviceState[device_index] = "0";
    digitalWrite(Device_pin[device_index], HIGH);
    String topic = "device-status/" + String(device_index + 1) + "/";
    client.publish(topic.c_str(), "0");
  }
}

void client_subscribe(void)
{
  // client.subscribe("command/1");
  // client.subscribe("command/2");
  // client.subscribe("command/3");
  // client.subscribe("command/4");
  // client.subscribe("command/5");
  // client.subscribe("command/6");
  client.subscribe("command/#");
  client.subscribe("change-mode/1/"); // change-mode/{node-id}
}

void client_publish_status(void)
{
  client.publish("node-status/1/", "1");
}

void client_loop(void)
{
  client.loop();
}

void init_control_node(void)
{
  // Initialize the GPIO pins for the devices as outputs and set them to HIGH (NC state)
  for (int i = device_1; i <= device_6; i++)
  {
    pinMode(Device_pin[i], OUTPUT);
    // Set initial relay states
    digitalWrite(Device_pin[i], DeviceState[i] == "1" ? LOW : HIGH);
  }
}

void TurnOnAutoMode(void){
  autoMode = 1;
  client.publish("mode-node/1/","1");
}

void TurnOffAutoMode(void){
  autoMode = 0;
  client.publish("mode-node/1/","0");
}
