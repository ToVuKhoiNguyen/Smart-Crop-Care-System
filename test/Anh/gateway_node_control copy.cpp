/* Define MQTT host */
#define DEFAULT_MQTT_HOST "mqtt1.eoh.io"

// You should get Auth Token in the ERa App or ERa Dashboard
#define ERA_AUTH_TOKEN "0ae424b1-ebd6-497c-9680-d19428d57778"

#include <LoRa.h>
#include <ERa.hpp>

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

void Transmit_Cmd(uint8_t value);

ERaTimer timer;

ERA_WRITE(V4)
{
    int value = param.getInt();
    Serial.print("Received command from web: ");
    Serial.println(value);
    Transmit_Cmd(uint8_t(value));
    ERa.virtualWrite(V4, value);
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

    /* Setup Client for Modbus TCP/IP */
    ERa.setModbusClient(mbTcpClient);

    /* Set scan WiFi. If activated, the board will scan
        and connect to the best quality WiFi. */
    ERa.setScanWiFi(true);
            
    /* Initializing the ERa library. */
    ERa.begin(ssid, pass);
}

void loop()
{
    ERa.run();
}

void Transmit_Cmd(uint8_t value)
{
    uint8_t buffer[8];

    buffer[0] = 0x4E;       // header
    buffer[1] = 0x57;       // header
    buffer[2] = 0x00;       // frame source
    buffer[3] = 0x02;       // command: 0x02-write
    buffer[4] = 1;          // device id (relay 13)
    buffer[5] = value;      // value to write
    buffer[6] = 0x00;       // checksum
    buffer[7] = 0x00;       // checksum

    LoRa.beginPacket();
    LoRa.write(buffer, sizeof(buffer));
    LoRa.endPacket();
}
