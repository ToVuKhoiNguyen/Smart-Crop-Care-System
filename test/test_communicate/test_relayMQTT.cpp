#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>

// Replace with your network credentials
const char *ssid = "Phong209";     // Network SSID (name)
const char *password = "pttkdtvt"; // Network password

// MQTT Broker details
const char *mqttServer = "112.137.129.171";
const int mqttPort = 1883;
const char *mqttUser = "";
const char *mqttPassword = "";

// Initialize the WiFi and MQTT client objects
WiFiClient espClient;
PubSubClient client(espClient);

// Variables to store the current state of each device (ON/OFF)
String DeviceState[6] = {"off", "off", "off", "off", "off", "off"};
String MQTT_topic[6] = {"relay/device1", "relay/device2", "relay/device3", 
                        "relay/device4", "relay/device5", "relay/device6"};

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
uint8_t Device_pin[6] = {25, 26, 27, 14, 12, 33};

// Variable to enable or disable state saving
const bool saveState = false;

// EEPROM address to store the state
const int eepromSize = 6;

// Function to handle arriving message
void handle_message(String message, int device);

// Callback function for MQTT subscription
void callback(char *topic, byte *message, unsigned int length);

void setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    // Initialize EEPROM
    if (saveState)
    {
        EEPROM.begin(eepromSize);
    }

    // Initialize the GPIO pins for the devices as outputs and set them to HIGH (NC state)
    for(int i = device_1; i < device_6; i++)
    {
        pinMode(Device_pin[i], OUTPUT);
        // Load saved states from EEPROM
        if(saveState)
            DeviceState[i] = EEPROM.read(0) == 1 ? "on" : "off";
        // Set initial relay states
        digitalWrite(Device_pin[i], DeviceState[i] == "on" ? LOW : HIGH);
    }

    // Connect to Wi-Fi
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // Connect to MQTT Broker
    client.setServer(mqttServer, mqttPort);
    client.setCallback(callback);

    while (!client.connected())
    {
        Serial.println("Connecting to MQTT...");
        if (client.connect("ESP32Client", mqttUser, mqttPassword))
        {
            Serial.println("connected");
        }
        else
        {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }
    digitalWrite(LED_BUILTIN, HIGH);

    // Subscribe to topics
    // for(int i = device_1; i < device_6; i++)
    //     client.subscribe(MQTT_topic[i]);
    client.subscribe("relay/device1");
    client.subscribe("relay/device2");
    client.subscribe("relay/device3");
    client.subscribe("relay/device4");
    client.subscribe("relay/device5");
    client.subscribe("relay/device6");
}

void loop()
{
    client.loop();
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

    if (saveState)
    {
        EEPROM.commit();
    }
}

void handle_message(String message, int device_index)
{
    if (message == "on")
    {
        Serial.printf("Device %d on", device_index + 1);
        DeviceState[device_index] = "on";
        digitalWrite(Device_pin[device_index], LOW);
        if (saveState)
            EEPROM.write(device_index, 1);
    }
    else if (message == "off")
    {
        Serial.printf("Device %d off", device_index + 1);
        DeviceState[device_index] = "off";
        digitalWrite(Device_pin[device_index], HIGH);
        if (saveState)
            EEPROM.write(device_index, 1);
    }
}