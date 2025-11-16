/*
 Basic ESP8266 MQTT example
 This sketch demonstrates the capabilities of the pubsub library in combination
 with the ESP8266 board/library.
 It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic" every two seconds
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary
  - If the first character of the topic "inTopic" is an 1, switch ON the ESP Led,
    else switch it off
 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.
 To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <strings.h>
#include "DFRobot_PH.h"
#include "DFRobot_EC.h"
#include <EEPROM.h>
#include <Adafruit_ADS1X15.h>
// Update these with values suitable for your network.

const char* ssid = "Vuon Phuc Loi";
const char* password = "vuonphucloi";
const char* mqtt_server = "20.189.115.59";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(100)
char msg[MSG_BUFFER_SIZE];
char ecMsg[MSG_BUFFER_SIZE];
char phMsg[MSG_BUFFER_SIZE];
int value = 0;
//setup ec and ph
#define PH_PIN 35

float  voltagePH,voltageEC,phValue;
float ecValue;
Adafruit_ADS1115 ads;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("command/Pump/");
      // client.subscribe("inTopic_relay2");
      // client.subscribe("inTopic_relay3");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {

  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  //connect to WiFi
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println(" CONNECTED");
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  //voltagePH = analogRead(PH_PIN)/1024.0*5000;          // read the ph voltage
  phValue    = (float)analogRead(PH_PIN)/4096*14;
  //voltageEC = analogRead(EC_PIN)/1024.0*5000;
  ecValue    = (float)ads.readADC_SingleEnded(0);

  unsigned long now = millis(); 
  if (now - lastMsg > 30000) {
    lastMsg = now;
    ++value;
    snprintf (ecMsg, MSG_BUFFER_SIZE, "%.1f", ecValue);
    snprintf (phMsg, MSG_BUFFER_SIZE, "%.1f", phValue);
    snprintf(msg, MSG_BUFFER_SIZE, "1");
    Serial.print("Publish message: ");
    Serial.println(phValue);
    Serial.println(ecValue);
    client.publish("sensors/2/", phMsg);
    client.publish("sensors/3/", ecMsg);
  }
}
