#include "LoRa_config.h"
#include "stdint.h"
#include <LoRa.h>

void init_LoRa()
{
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
    while (!LoRa.begin(433E6))
    {
        Serial.print("Waiting for LoRa initalization");
        Serial.print(".");
        delay(1000);
    }
    LoRa.setSyncWord(0xA5);
    Serial.println("LoRa Initializing OK!");
}

// Hàm gửi lệnh relay qua LoRa
void Transmit_Cmd(uint8_t relay, uint8_t value)
{
    uint8_t buffer[8];

    buffer[0] = 0x4E;  // header
    buffer[1] = 0x57;  // header
    buffer[2] = 0x00;  // frame source
    buffer[3] = 0x02;  // command: 0x02-write
    buffer[4] = relay; // device id (relay)
    buffer[5] = value; // value to write
    buffer[6] = 0x00;  // checksum
    buffer[7] = 0x00;  // checksum

    LoRa.beginPacket();
    LoRa.write(buffer, sizeof(buffer));
    LoRa.endPacket();
}

void run_LoRa()
{
    int packetSize = LoRa.parsePacket();
    if (packetSize)
    {
        Serial.print("Packet size: ");
        Serial.println(packetSize);

        uint8_t buffer[18]; // Tăng kích thước buffer để nhận thêm dữ liệu
        int bytesRead = 0;
        while (LoRa.available() && bytesRead < packetSize)
        {
            buffer[bytesRead++] = LoRa.read();
        }

        Serial.print("Received bytes: ");
        for (int i = 0; i < packetSize; i++)
        {
            Serial.print(buffer[i], HEX);
            Serial.print(" ");
        }
        Serial.println();

        // Kiểm tra header và lệnh
        if (buffer[0] == 0x4E && buffer[1] == 0x57 && buffer[3] == 0x01)
        {
            // Giải mã dữ liệu
            uint16_t tempInt = (buffer[4] << 8) | buffer[5];
            uint16_t humInt = (buffer[6] << 8) | buffer[7];
            uint16_t soilMoistureInt = (buffer[8] << 8) | buffer[9];
            uint16_t voltageInt = (buffer[10] << 8) | buffer[11];
            uint16_t currentInt = (buffer[12] << 8) | buffer[13];
            uint16_t ecInt = (buffer[14] << 8) | buffer[15]; // Dữ liệu EC
            uint16_t phInt = (buffer[16] << 8) | buffer[17]; // Dữ liệu pH

            float temperature = tempInt / 10.0;
            float humidity = humInt / 10.0;
            float soilMoisture = soilMoistureInt / 10.0;
            float voltage = voltageInt / 100.0;
            float current = currentInt / 100.0;
            float ec = ecInt / 100.0; // Chuyển đổi giá trị EC
            float ph = phInt / 100.0; // Chuyển đổi giá trị pH

            Serial.print("Received - Temp: ");
            Serial.print(temperature);
            Serial.print(" °C, Hum: ");
            Serial.print(humidity);
            Serial.print(" %, Soil Moisture: ");
            Serial.print(soilMoisture);
            Serial.print(" %, Voltage: ");
            Serial.print(voltage);
            Serial.print(" V, Current: ");
            Serial.print(current);
            Serial.print(" mA, EC: ");
            Serial.print(ec);
            Serial.print(" mS/cm, pH: ");
            Serial.println(ph);

            // // Gửi lên ERA IoT
            // ERa.virtualWrite(V0, temperature);
            // ERa.virtualWrite(V1, humidity);
            // ERa.virtualWrite(V2, soilMoisture);
            // ERa.virtualWrite(V5, voltage);
            // ERa.virtualWrite(V6, current);
            // ERa.virtualWrite(V7, ec); // Gửi EC lên V7
            // ERa.virtualWrite(V8, ph); // Gửi pH lên V8
        }
    }
}
