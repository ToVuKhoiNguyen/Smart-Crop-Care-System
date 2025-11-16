#include <LoRa.h>
#include <Wire.h>
#include <sensor_config.h>

#define ss 5
#define rst 4
#define dio0 2

uint8_t buffer[8] = {0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x07, 0x18};

int counter = 0;

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    while (!Serial);
    Serial.println("LoRa Sender");

    LoRa.setPins(ss, rst, dio0); // setup LoRa transceiver module

    while (!LoRa.begin(433E6)) // 433E6 - Asia, 866E6 - Europe, 915E6 - North America
    {
        Serial.print(".");
        delay(500);
    }
    LoRa.setSyncWord(0xA5);
    Serial.println("LoRa Initializing OK!");
}

void loop()
{

    // float temperature = get_temperature();
    // float humidity = get_humidity();
    Serial.println("Sending packet: ");
    // Serial.println(counter);
    // Serial.print("Temperature: ");
    // Serial.print(temperature);
    // Serial.println("ยบC");
    // Serial.println("");

    // LoRa.beginPacket(); // Send LoRa packet to receiver
    // LoRa.print("Pckt: ");
    // LoRa.println(counter);
    // LoRa.print("Temp: ");
    // LoRa.print(temperature);
    // LoRa.println(" C");
    // LoRa.print("Humid: ");
    // LoRa.print(humidity);
    // LoRa.println(" %");
    // LoRa.endPacket();

    // counter++;
    LoRa.beginPacket();
    LoRa.write(buffer, sizeof(buffer));
    LoRa.endPacket();
    delay(4000);
}