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
#include "time.h"
#include "sntp.h"

// Update these with values suitable for your network.

const char* ssid = "Vuon Phuc Loi";
const char* password = "vuonphucloi";
const char* mqtt_server = "20.189.115.59";
//time UTC
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

const char* time_zone = "ICT-7";  // TimeZone rule for Europe/Rome including daylight adjustment rules (optional)
int hour = -1;
int minute=-1;
int second=-1;
int count = 0;
const int RELAY_1 = 15;
const int RELAY_2 = 26;
const int RELAY_3 = 27;
//
int status_relay1 = 0;
int status_relay2 = 1; // dau nham relay2 va relay 3
int status_relay3 = 1;
int autoMode = 1; // 1-auto 0-manual
//
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(100)
char msg[MSG_BUFFER_SIZE];
char str[100];
int pump_duration = 2; // 1 tiếng bơm 3 phút
// //setup time
void updateTime()
{
  
  struct tm time;
  if(!getLocalTime(&time)){
    Serial.print("No time available (yet)");
    return;
  }

  
  hour = time.tm_hour;
  minute = time.tm_min;
  second = time.tm_sec;
  // snprintf (str, 100, "%d:%d:%d",time.tm_hour, time.tm_min, time.tm_sec);
  // Serial.println(str);
}

// Callback function (get's called when time adjusts via NTP)
void timeavailable(struct timeval *t)
{
  Serial.println("Got time adjustment from NTP!"); 
  updateTime();
}

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

  // Switch on the LED if an 1 was received as first character
  if(strcmp(topic,"command/1/") == 0){
    if ((char)payload[0] == '1') {
      TurnOnRelay(RELAY_1);
    } else {
      TurnOffRelay(RELAY_1);
    }
    return;
  }
  if(strcmp(topic,"command/2/") == 0){
    if ((char)payload[0] == '1') {
      TurnOnRelay(RELAY_2);
    } else {
      TurnOffRelay(RELAY_2);
    }
    return;
  }
  
  if(strcmp(topic,"command/3/") == 0){
    if ((char)payload[0] == '1') {
      TurnOnRelay(RELAY_3);
    } else {
      TurnOffRelay(RELAY_3);
    }
    return;
  }
  if(strcmp(topic,"change-mode/1/") == 0){
    if ((char)payload[0] == '1') {
      TurnOnAutoMode();
    } else {
      TurnOffAutoMode();
    }
    return;
  }
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
      client.publish("node-status/1/", "1");
      // ... and resubscribe
      client.subscribe("command/#");
      client.subscribe("change-mode/1/"); // change-mode/{node-id}
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
  pinMode(RELAY_1, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  pinMode(RELAY_2, OUTPUT); 
  pinMode(RELAY_3, OUTPUT); 
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  digitalWrite(RELAY_1, HIGH);
  digitalWrite(RELAY_2, HIGH);
  digitalWrite(RELAY_3, HIGH);

  // set notification call-back function
  sntp_set_time_sync_notification_cb( timeavailable );

  /**
   * NTP server address could be aquired via DHCP,
   *
   * NOTE: This call should be made BEFORE esp32 aquires IP address via DHCP,
   * otherwise SNTP option 42 would be rejected by default.
   * NOTE: configTime() function call if made AFTER DHCP-client run
   * will OVERRIDE aquired NTP server address
   */
  sntp_servermode_dhcp(1);    // (optional)

  /**
   * This will set configured ntp servers and constant TimeZone/daylightOffset
   * should be OK if your time zone does not need to adjust daylightOffset twice a year,
   * in such a case time adjustment won't be handled automagicaly.
   */
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

  /**
   * A more convenient approach to handle TimeZones with daylightOffset 
   * would be to specify a environmnet variable with TimeZone definition including daylight adjustmnet rules.
   * A list of rules for your zone could be obtained from https://github.com/esp8266/Arduino/blob/master/cores/esp8266/TZ.h
   */
  configTzTime(time_zone, ntpServer1, ntpServer2);

  //connect to WiFi
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println(" CONNECTED");

}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    setup_wifi();
  }

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // unsigned long now = millis();
  // if (now - lastMsg > 10000) {
  //   lastMsg = now;
  //   ++value;
  //   // snprintf (msg, MSG_BUFFER_SIZE, " Status_relay1 :%d,Status_relay2 :%d,Status_relay3 :%d, id: %d ",status_relay1,status_relay2,status_relay3,value);
  //   snprintf(msg, MSG_BUFFER_SIZE, "1");
  //   Serial.print("Publish message: ");
  //   Serial.println(timer.tm_hour) 
  //   printLocalTime(); 
  //   Serial.println(msg);
  //   client.publish("node-status/1/", msg);
  // }
  updateTime();
  snprintf (str, 100, "%d:%d:%d",hour, minute, second);
  //Serial.println(str);
  if(count % 2 == 0){
    // client.publish("node-status/1/","1");
    if(autoMode==1){
      client.publish("mode-node/1/","1");
    }
    else{
      client.publish("mode-node/1/","0");
      }
    status_relay1 == 1 ? client.publish("device-status/1/", "1") : client.publish("device-status/1/", "0");
    status_relay2 == 0 ? client.publish("device-status/2/", "1") : client.publish("device-status/2/", "0"); // dau nham
    status_relay3 == 0 ? client.publish("device-status/3/", "1") : client.publish("device-status/3/", "0"); // dau nhAM
  }
  if (autoMode == 1){
    if(hour >= 8 && hour <= 12 && minute >= 0 && minute <= pump_duration){
      //Serial.println("hour >= 8 && hour <= 12 && minute >= 0 && minute <= pump_duration");
      TurnOnRelay(RELAY_1);
      TurnOnRelay(RELAY_2);
      TurnOnRelay(RELAY_3);
    }
    else if(hour >= 13 && hour <= 17 && minute >= 0 && minute <= pump_duration){
      //Serial.println("hour >= 13 && hour <= 18 && minute <= 0 && minute <= pump_duration");
      TurnOnRelay(RELAY_1);
      TurnOnRelay(RELAY_2);
    }
    else{
      TurnOffRelay(RELAY_1);
      TurnOffRelay(RELAY_2);
      TurnOffRelay(RELAY_3);
    }
  }
  
  count++;
  delay(1000);
}
void TurnOnRelay(const int relay){
  
  if(relay == RELAY_1){
    client.publish("device-status/1/","1");
    status_relay1 = 1;
    digitalWrite(relay, LOW);
  }
  else if(relay == RELAY_2){
    client.publish("device-status/2/","1");
    status_relay2 = 0; // dau nham 
    digitalWrite(relay, HIGH);
    }
  else if(relay == RELAY_3){
    client.publish("device-status/3/","1");
    status_relay3 = 0; // dau nham
    digitalWrite(relay, LOW);
  }
}
void TurnOffRelay(const int relay){
  
  if(relay == RELAY_1){
    client.publish("device-status/1/","0");
    status_relay1 = 0;
    digitalWrite(relay, HIGH);
  }
  else if(relay == RELAY_2){
    client.publish("device-status/2/","0");
    status_relay2 = 1; // dau nham
    digitalWrite(relay, LOW);
    }
  else if(relay == RELAY_3){
    client.publish("device-status/3/","0");
    status_relay3 = 1; // dau nham
    digitalWrite(relay, LOW);
  }
}
void TurnOnAutoMode(){
  autoMode = 1;
  client.publish("mode-node/1/","1");
}
void TurnOffAutoMode(){
  autoMode = 0;
  client.publish("mode-node/1/","0");
}
