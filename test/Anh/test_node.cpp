/* Define MQTT host */
#define DEFAULT_MQTT_HOST "mqtt1.eoh.io"

// You should get Auth Token in the ERa App or ERa Dashboard
#define ERA_AUTH_TOKEN "b8d1a209-65ba-4d72-a6ad-b619d1c99355"

#include <LoRa.h>
#include <ERa.hpp>
#include <Era/ERaTimer.hpp>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 4     // Digital pin connected to the DHT sensor 

#define DHTTYPE    DHT11     // DHT 11
// #define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

DHT_Unified dht(DHTPIN, DHTTYPE);

const char ssid[] = "512B";
const char pass[] = "1123581321";

WiFiClient mbTcpClient;

/* This function will run every time ERa is connected */
ERA_CONNECTED() {
    ERA_LOG(ERA_PSTR("ERa"), ERA_PSTR("ERa connected!"));
}

/* This function will run every time ERa is disconnected */
ERA_DISCONNECTED() {
    ERA_LOG(ERA_PSTR("ERa"), ERA_PSTR("ERa disconnected!"));
}

void Transmit_Cmd(uint8_t deviceId, uint8_t value);

ERaTimer timer;

ERA_WRITE(V0)
{
    int value = param.getInt();
    // Transmit_Cmd(1, uint8_t(value));
    ERa.virtualWrite(V0, value);
}

ERA_WRITE(V1)
{
    int value = param.getInt();
    // Transmit_Cmd(2, uint8_t(value));
    ERa.virtualWrite(V1, value);
}

ERA_WRITE(V2)
{
    int value = param.getInt();
    // Transmit_Cmd(3, uint8_t(value));
    ERa.virtualWrite(V2, value);
}

ERA_WRITE(V3)
{
    int value = param.getInt();
    // Transmit_Cmd(4, uint8_t(value));
    ERa.virtualWrite(V3, value);
}

ERA_WRITE(V4)
{
    int value = param.getInt();
    // Transmit_Cmd(5, uint8_t(value));
    ERa.virtualWrite(V4, value);
}

ERA_WRITE(V5)
{
    int value = param.getInt();
    // Transmit_Cmd(6, uint8_t(value));
    ERa.virtualWrite(V5, value);
}

void timerEvent()
{
  Serial.println("Sent");
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  ERa.virtualWrite(V6, event.temperature);
  dht.humidity().getEvent(&event);
  ERa.virtualWrite(V7, event.relative_humidity);
}

#define ss 5
#define rst 4
#define dio0 2

void setup()
{
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Gateway node");
    LoRa.setPins(ss, rst, dio0);
    while (!LoRa.begin(433E6))
    {
        Serial.print("Waiting for LoRa initalization");
        Serial.print(".");
        delay(1000);
    }
    LoRa.setSyncWord(0xA5);
    Serial.println("LoRa Initializing OK!");
    // LoRa.end();

    /* Setup Client for Modbus TCP/IP */
    ERa.setModbusClient(mbTcpClient);

    /* Set scan WiFi. If activated, the board will scan
        and connect to the best quality WiFi. */
    ERa.setScanWiFi(true);
            
    /* Initializing the ERa library. */
    ERa.begin(ssid, pass);

    // Initialize device.
    dht.begin();
    Serial.println(F("DHTxx Unified Sensor Example"));

    timer.setInterval(1000L, timerEvent);
}

void loop()
{
    ERa.run();
    timer.run();
    
}

void Transmit_Cmd(uint8_t deviceId, uint8_t value)
{   
    uint8_t buffer[8];

    buffer[0] = 0x4E;       // header
    buffer[1] = 0x57;       // header
    buffer[2] = 0x00;       // frame source
    buffer[3] = 0x02;       // command: 0x02-write
    buffer[4] = deviceId;   // device id
    buffer[5] = value;      // value to write
    buffer[6] = 0x00;       // checksum
    buffer[7] = 0x00;        // checksum

    LoRa.beginPacket();
    LoRa.write(buffer, sizeof(buffer));
    LoRa.endPacket();
}