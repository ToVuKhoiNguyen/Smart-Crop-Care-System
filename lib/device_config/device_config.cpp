#include "device_config.h"
#include "sensor_config.h"
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <TFT_eSPI.h>    // Hardware-specific library
#include <JPEGDecoder.h> // JPEG decoder library
#include <LoRa.h>
#include <Wifi.h>
#include <PnP/ERaPnPEsp32.hpp>
#include <WiFi.h>

extern ERaPnP<ERaMqtt<WiFiClient, MQTTClient>> ERa;
extern RTC_DS3231 rtc;
extern char daysOfTheWeek[7][5];
extern sensor_data sensor;
extern sensor_state sensor_status;
extern bool isPHStable;

uint8_t sensor_node_mac_address[6];

bool RELAY_1[NUMBER_OF_CTRL_NODES] = {false, false};
bool RELAY_2[NUMBER_OF_CTRL_NODES] = {false, false};
bool RELAY_3[NUMBER_OF_CTRL_NODES] = {false, false};
bool RELAY_4[NUMBER_OF_CTRL_NODES] = {false, false};
bool RELAY_5[NUMBER_OF_CTRL_NODES] = {false, false};

void set_SPI_CS_pin()
{
    // Set all chip selects high to avoid bus contention during initialisation of each peripheral
    // pinMode(TOUCH_CS, OUTPUT); // Touch controller chip select (if used)
    // pinMode(TFT_CS, OUTPUT);   // TFT screen chip select
    // pinMode(SD_CS, OUTPUT);    // SD card chips select, must use GPIO 5 (ESP32 SS)
    // pinMode(LORA_CS, OUTPUT);  // LoRa chip select

    digitalWrite(TOUCH_CS, HIGH); // Touch controller chip select (if used)
    digitalWrite(TFT_CS, HIGH);   // TFT screen chip select
    digitalWrite(SD_CS, HIGH);    // SD card chips select, must use GPIO 5 (ESP32 SS)
    digitalWrite(LORA_CS, HIGH);  // LoRa chip select
}

/* SD Card*/

void init_sd_card()
{
    Serial.println("Init SD Card");
    if (!SD.begin(SD_CS))
    {
        Serial.println("Lỗi khi khởi tạo thẻ SD!");
        return;
    }
    Serial.println("Thẻ SD đã sẵn sàng.");
}

void logDataToSD(float temp, float humid, float ec, float ph, float volt, float curr, float light, int moisture)
{
    DateTime now = rtc.now();

    // Tạo tên file dạng "2025-05-15.csv"
    char filename[20];
    sprintf(filename, "/%04d-%02d-%02d.csv", now.year(), now.month(), now.day());

    // Mở file ở chế độ append
    File logFile = SD.open(filename, FILE_APPEND);
    if (logFile)
    {
        // Ghi dòng dữ liệu dạng: hh:mm:ss, temp, humid, ec, ph, volt, curr, light
        logFile.print(now.hour());
        logFile.print(":");
        logFile.print(now.minute());
        logFile.print(":");
        logFile.print(now.second());
        logFile.print(", ");
        logFile.print(temp);
        logFile.print(", ");
        logFile.print(humid);
        logFile.print(", ");
        logFile.print(ec);
        logFile.print(", ");
        logFile.print(ph);
        logFile.print(", ");
        logFile.print(volt);
        logFile.print(", ");
        logFile.print(curr);
        logFile.print(", ");
        logFile.print(light);
        logFile.print(", ");
        logFile.println(moisture);
        logFile.close();
        Serial.println("Đã ghi dữ liệu vào thẻ nhớ.");
    }
    else
    {
        Serial.println("Không mở được file log.");
    }
}

void get_sd_card_type()
{
    uint8_t cardType = SD.cardType();

    if (cardType == CARD_NONE)
    {
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC)
    {
        Serial.println("MMC");
    }
    else if (cardType == CARD_SD)
    {
        Serial.println("SDSC");
    }
    else if (cardType == CARD_SDHC)
    {
        Serial.println("SDHC");
    }
    else
    {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

/* LoRa*/
// TFT_eSPI tft = TFT_eSPI(); // Invoke custom library
bool hasIncomingPacket = false;
QueueHandle_t loraQueue;
int packageRSSI = 0;
float packageSNR = 0.0;
bool ackReceived = false;

void loraInit(void)
{
    Serial.println("Lora Init");
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0); // Set CS, RST, IRQ pin
    while (!LoRa.begin(433E6))
    {
        Serial.print("Waiting for LoRa initialization");
        Serial.print("...");
        delay(1000);
    }

    Serial.println("LoRa Initializing OK!");
}

void loraConfig(int fr, int pw, int sf, int bw, int sw, int cr)
{
    Serial.println("Lora Config");
    LoRa.setFrequency(fr);       // Set frequency to 433E6 Hz
    LoRa.setTxPower(pw);         // Set TX power to 14 dBm: 14-20 dBm. Tăng công suất phát sẽ tăng khoảng cách truyền nhưng cũng làm giảm thời gian sử dụng pin.
    LoRa.setSpreadingFactor(sf); // Set spreading factor to 7: 6-12. Hệ số trải phổ càng lớn thì khoảng cách truyền càng xa nhưng tốc độ truyền càng chậm.
    LoRa.setSignalBandwidth(bw); // Set bandwidth to 125E3 Hz: từ 7.8 kHz đến 500 kHz. Băng thông càng lớn cho phép tốc độ dữ liệu cao hơn và thời gian lưu thông tín hiệu trên không gian (time on air) thấp hơn, nhưng làm giảm độ nhạy thu do nhiễu tăng.
    LoRa.setSyncWord(sw);        // Set sync word to 0xA5
    LoRa.setCodingRate4(cr);     // Set coding rate to 4/5: Tốc độ mã hoá
    Serial.println("Success");
}

void createQueue(void)
{
    loraQueue = xQueueCreate(10, sizeof(Packet));
    if (!loraQueue)
    {
        Serial.println("❌ Failed to create Queue");
        while (1)
            ;
    }
}

void loraReadPackage(void)
{
    int packetSize = LoRa.parsePacket();
    if (packetSize)
    {
        packageRSSI = LoRa.packetRssi();
        packageSNR = LoRa.packetSnr();
        Packet pkt;
        pkt.len = packetSize;
        for (int i = 0; i < packetSize && i < MAX_PACKET_SIZE; i++)
        {
            pkt.data[i] = LoRa.read();
        }

        if (xQueueSend(loraQueue, &pkt, 0) != pdPASS)
        {
            Serial.println("⚠️ Queue full, packet dropped");
        }
        else
        {
            Serial.println("📥 Packet pushed to queue");
        }
    }
}

#if defined(GATEWAY_NODE)
void loraProcessPackage(void)
{
    Packet pkt;
    if (xQueueReceive(loraQueue, &pkt, portMAX_DELAY) == pdTRUE)
    {
        Serial.print("📦 Received packet: ");
        for (int i = 0; i < pkt.len; i++)
        {
            Serial.printf("%02X ", pkt.data[i]);
        }
        Serial.println();
        decodeIncomingPacket(pkt.data, pkt.len);
    }
}
#endif

#if defined(SENSOR_NODE)
bool sendSensorData(const uint8_t *mac_address)
{
    // kiểm tra số lượng cảm biến trước khi gửi dữ liệu
    uint8_t count = check_sensor_status();
    Serial.printf("Active sensor: %d\n", count);

    // buffer sẽ có 14 bytes cố định, thêm 3 * count bytes cho các cảm biến
    uint8_t length = NUMBER_OF_FIXED_BYTES + 3 * count;
    uint8_t buffer[length];
    uint8_t bufferIndex = 0;
    memset(buffer, 0x00, sizeof(buffer)); // Reset buffer (gán tất cả phần tử trong buffer về 0x00 hoặc 0xFF)

    // đọc dữ liệu cảm biến
    // read_sensor_node_data();

    // Thêm header vào buffer
    buffer[bufferIndex++] = STX1; // N
    buffer[bufferIndex++] = STX2; // W
    buffer[bufferIndex++] = length >> 8;
    buffer[bufferIndex++] = length & 0xFF;
    for (int i = 0; i < NUMBER_OF_MAC_BYTES; i++)
        buffer[bufferIndex++] = mac_address[i];
    buffer[bufferIndex++] = SENSOR_VAL; // Lệnh gửi dữ liệu

    if (sensor_status.voltage_sensor)
    {
        buffer[bufferIndex++] = VOLTAGE_SENSOR;
        uint16_t voltVal = (uint16_t)(sensor.voltage * 100);
        buffer[bufferIndex++] = voltVal >> 8;
        buffer[bufferIndex++] = voltVal & 0xFF;
    }

    if (sensor_status.current_sensor)
    {
        buffer[bufferIndex++] = CURRENT_SENSOR;
        uint16_t currVal = (uint16_t)(sensor.current * 100);
        buffer[bufferIndex++] = currVal >> 8;
        buffer[bufferIndex++] = currVal & 0xFF;
    }

    if (sensor_status.light_sensor)
    {
        buffer[bufferIndex++] = LIGHT_SENSOR;
        uint16_t lightVal = (uint16_t)sensor.lumiorsity_value;
        buffer[bufferIndex++] = lightVal >> 8;
        buffer[bufferIndex++] = lightVal & 0xFF;
    }

    if (sensor_status.temperature_sensor)
    {
        buffer[bufferIndex++] = TEMPERATURE_SENSOR;
        uint16_t tempVal = (uint16_t)(sensor.temperature * 100);
        buffer[bufferIndex++] = tempVal >> 8;
        buffer[bufferIndex++] = tempVal & 0xFF;
    }

    if (sensor_status.humidity_sensor)
    {
        buffer[bufferIndex++] = HUMIDITY_SENSOR;
        uint16_t humVal = (uint16_t)(sensor.humidity * 100);
        buffer[bufferIndex++] = humVal >> 8;
        buffer[bufferIndex++] = humVal & 0xFF;
    }

    if (sensor_status.soil_moisture_sensor)
    {
        buffer[bufferIndex++] = SOIL_MOISTURE_SENSOR;
        uint16_t soilVal = (uint16_t)sensor.soil_moisture;
        buffer[bufferIndex++] = soilVal >> 8;
        buffer[bufferIndex++] = soilVal & 0xFF;
    }

    if (sensor_status.ec_sensor)
    {
        buffer[bufferIndex++] = EC_SENSOR;
        uint16_t ecVal = (uint16_t)(sensor.ec_value * 100);
        buffer[bufferIndex++] = ecVal >> 8;
        buffer[bufferIndex++] = ecVal & 0xFF;
    }

    if (sensor_status.ph_sensor)
    {
        // cần chờ pH ổn định thì mới gửi, không thì huỷ
        if (!isPHStable)
        {
            Serial.println("pH is not stable, can not send data");
            return false;
        }
        buffer[bufferIndex++] = PH_SENSOR;
        uint16_t phVal = (uint16_t)(sensor.ph_value * 100);
        buffer[bufferIndex++] = phVal >> 8;
        buffer[bufferIndex++] = phVal & 0xFF;
    }

    buffer[bufferIndex++] = END_BYTE; // End identification
    // 2 bytes cuối là checksum, sử dụng CRC16
    uint16_t crc = CalculateCRC(buffer, length - 2);
    buffer[bufferIndex++] = (crc >> 8) & 0xFF; // CRC High
    buffer[bufferIndex++] = crc & 0xFF;        // CRC Low

    // In các byte của buffer qua Serial trước khi gửi
    Serial.print("Sending LoRa Data: ");
    for (int i = 0; i < length; i++)
    {
        Serial.print(buffer[i], HEX); // In ra giá trị mỗi byte dưới dạng hex
        Serial.print(" ");
    }
    Serial.println(); // Xuống dòng sau khi in hết các byte

    // Gửi buffer đủ 20 byte
    LoRa.beginPacket();
    LoRa.write(buffer, length);
    LoRa.endPacket();
    LoRa.idle();
    Serial.println("LoRa packet sent.");

    return true;
}

void sendSensorConfig(const uint8_t *mac_address)
{
    uint8_t count = check_sensor_status(); // Đếm số cảm biến đang hoạt động
    Serial.printf("Active sensors: %d\n", count);

    // Tổng số cảm biến hỗ trợ là 8 → dữ liệu cảm biến luôn là 2 * 8 = 16 byte
    uint8_t length = NUMBER_OF_FIXED_BYTES + 2 * 8; // Fixed length
    uint8_t buffer[length];
    uint8_t bufferIndex = 0;
    memset(buffer, 0x00, sizeof(buffer));

    buffer[bufferIndex++] = STX1; // N
    buffer[bufferIndex++] = STX2; // W
    buffer[bufferIndex++] = length >> 8;
    buffer[bufferIndex++] = length & 0xFF;
    for (int i = 0; i < NUMBER_OF_MAC_BYTES; i++)
        buffer[bufferIndex++] = mac_address[i];
    buffer[bufferIndex++] = CONFIG_SEN; // Lệnh gửi dữ liệu

    buffer[bufferIndex++] = VOLTAGE_SENSOR;
    buffer[bufferIndex++] = sensor_status.voltage_sensor ? 1 : 0;
    buffer[bufferIndex++] = CURRENT_SENSOR;
    buffer[bufferIndex++] = sensor_status.current_sensor ? 1 : 0;
    buffer[bufferIndex++] = LIGHT_SENSOR;
    buffer[bufferIndex++] = sensor_status.light_sensor ? 1 : 0;
    buffer[bufferIndex++] = TEMPERATURE_SENSOR;
    buffer[bufferIndex++] = sensor_status.temperature_sensor ? 1 : 0;
    buffer[bufferIndex++] = HUMIDITY_SENSOR;
    buffer[bufferIndex++] = sensor_status.humidity_sensor ? 1 : 0;
    buffer[bufferIndex++] = SOIL_MOISTURE_SENSOR;
    buffer[bufferIndex++] = sensor_status.soil_moisture_sensor ? 1 : 0;
    buffer[bufferIndex++] = EC_SENSOR;
    buffer[bufferIndex++] = sensor_status.ec_sensor ? 1 : 0;
    buffer[bufferIndex++] = PH_SENSOR;
    buffer[bufferIndex++] = sensor_status.ph_sensor ? 1 : 0;

    buffer[bufferIndex++] = END_BYTE; // End identification
    uint16_t crc = CalculateCRC(buffer, length - 2);
    buffer[bufferIndex++] = (crc >> 8) & 0xFF; // CRC High
    buffer[bufferIndex++] = crc & 0xFF;        // CRC Low

    Serial.print("🔼 Sent SENSOR CONFIG Packet: ");
    for (int i = 0; i < length; i++)
        Serial.printf("%02X ", buffer[i]);
    Serial.println();

    LoRa.beginPacket();
    LoRa.write(buffer, length);
    LoRa.endPacket();
    // LoRa.receive();
    LoRa.idle();
    Serial.println("🕒 Chờ ACK");
    if (!waitForAck(5000, mac_address))
    {
        Serial.println("❌ ACK timeout");
    }
    else
    {
        Serial.println("✅ ACK received!");
    }
}

void decodeSensorPacket(uint8_t *data, uint8_t incomingLength, const uint8_t *macAdress)
{
    // Kiểm tra header
    if (data[0] != STX1 || data[1] != STX2)
    {
        Serial.println("Invalid packet header!");
        return;
    }
    // Kiểm tra độ dài gói tin
    uint16_t length = (data[2] << 8) | data[3];
    if (length != incomingLength)
    {
        Serial.println("Invalid packet length!");
        Serial.println(length);
        Serial.println(incomingLength);
        return;
    }
    // Kiểm tra CRC
    uint16_t crc = CalculateCRC(data, length - 2);
    if (crc != ((data[length - 2] << 8) | data[length - 1]))
    {
        // Serial.println(crc, HEX);
        Serial.println("Invalid CRC!");
        return;
    }
    // kiểm tra end byte (0x68)
    if (data[length - 3] != END_BYTE)
    {
        Serial.println("Invalid end byte!");
        return;
    }
    // So sánh MAC
    for (int i = 0; i < NUMBER_OF_MAC_BYTES; i++)
    {
        if (data[4 + i] != macAdress[i])
        {
            Serial.println("MAC addresses are not equal.");
            return;
        }
    }
    // cấu hình lệnh
    uint8_t packetType = data[10];
    switch (packetType)
    {
    case CONFIG_REQ:
        sendSensorConfig(macAdress);
        break;
    case SENSOR_VAL:
        break;
    case ACK:
        if (data[11] == 'O' && data[12] == 'K')
        {
            ackReceived = true;
        }
        else
            Serial.println("❌ Invalid message, not OK");
        break;
    default:
        Serial.printf("⚠️ Package Type không xác định: %d\n", packetType);
        break;
    }
}
#endif

#if defined(GATEWAY_NODE)
// hàm này sẽ giải mã tất cả các loại gói tin
// từ đó mới chia ra xử lý các loại gói tin
void decodeIncomingPacket(uint8_t *data, uint8_t incomingLength)
{
    // Kiểm tra header
    if (data[0] != STX1 || data[1] != STX2)
    {
        Serial.println("Invalid packet header!");
        return;
    }
    // Kiểm tra độ dài gói tin
    uint16_t length = (data[2] << 8) | data[3];
    if (length != incomingLength)
    {
        Serial.println("Invalid packet length!");
        Serial.println(length);
        Serial.println(incomingLength);
        return;
    }
    // Kiểm tra CRC
    uint16_t crc = CalculateCRC(data, length - 2);
    if (crc != ((data[length - 2] << 8) | data[length - 1]))
    {
        // Serial.println(crc, HEX);
        Serial.println("Invalid CRC!");
        return;
    }
    // kiểm tra end byte (0x68)
    if (data[length - 3] != END_BYTE)
    {
        Serial.println("Invalid end byte!");
        return;
    }
    // cấu hình lệnh
    uint8_t packetType = data[10];
    // xử lý địa chỉ mac ở trong các hàm dưới đây
    switch (packetType)
    {
    case SENSOR_VAL:
        handleSensorData(data, length);
        break;
    case CONFIG_REQ:
        handleControlPacket(data, length, packetType);
        break;
    case STATUS:
        handleControlPacket(data, length, packetType);
        break;
    case CONFIG_SEN:
        handleSensorConfig(data, length);
        break;
    default:
        Serial.printf("⚠️ Package Type không xác định: %d\n", packetType);
        break;
    }
}

sensor_data sensor_node_data[NUMBER_OF_SENS_NODES];

// hàm này sẽ gửi dữ liệu cảm biến lên ERa IoT
void handleSensorData(uint8_t *data, uint8_t length)
{
    // kiểm tra địa chỉ mac để lấy thứ tự cảm biến
    uint8_t mac[NUMBER_OF_MAC_BYTES];
    for (int i = 0; i < NUMBER_OF_MAC_BYTES; i++)
    {
        mac[i] = data[4 + i]; // lấy địa chỉ mac
    }
    uint8_t order = 255;
    for (int i = 0; i < NUMBER_OF_SENS_NODES; i++)
    {
        if (memcmp(mac, mac_sens_node[i], NUMBER_OF_MAC_BYTES) == 0)
        {
            order = i;
            break;
        }
    }
    if (order == 255)
    {
        Serial.println("Invalid MAC address!");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    ; // để đảm bảo gửi xong
    Serial.println("Sending ACK...");
    sendNodePacket(ACK, mac);
    Serial.print("ACK sent to MAC: ");
    for (int i = 0; i < NUMBER_OF_MAC_BYTES; i++)
    {
        Serial.printf("%02X", mac[i]);
        if (i < 5)
            Serial.print(":");
    }
    Serial.println();
    uint8_t index = 11; // dữ liệu bắt đầu từ byte thứ 11
    uint8_t dataLength = length - NUMBER_OF_FIXED_BYTES + index;
    // Serial.printf("data length = %d", dataLength);
    while (index != dataLength)
    {
        uint16_t sensorValue = (data[index + 1] << 8) | data[index + 2];
        float sensorValueFloat = 0.0;
        if (data[index] == LIGHT_SENSOR || data[index] == SOIL_MOISTURE_SENSOR)
        {
            sensorValueFloat = float(sensorValue);
        }
        else
            sensorValueFloat = sensorValue / 100.0;
        // cập nhật vào struct
        switch (data[index])
        {
        case VOLTAGE_SENSOR:
            sensor_node_data[order].voltage = sensorValueFloat;
            break;
        case CURRENT_SENSOR:
            sensor_node_data[order].current = sensorValueFloat;
            break;
        case LIGHT_SENSOR:
            sensor_node_data[order].lumiorsity_value = sensorValueFloat;
            break;
        case TEMPERATURE_SENSOR:
            sensor_node_data[order].temperature = sensorValueFloat;
            break;
        case HUMIDITY_SENSOR:
            sensor_node_data[order].humidity = sensorValueFloat;
            break;
        case SOIL_MOISTURE_SENSOR:
            sensor_node_data[order].soil_moisture = sensorValueFloat;
            break;
        case EC_SENSOR:
            sensor_node_data[order].ec_value = sensorValueFloat;
            break;
        case PH_SENSOR:
            sensor_node_data[order].ph_value = sensorValueFloat;
            break;
        default:
            break;
        }
        // Ghi vào thẻ nhớ
        logDataToSD(sensor_node_data[order].temperature, 
                    sensor_node_data[order].humidity, 
                    sensor_node_data[order].ec_value, 
                    sensor_node_data[order].ph_value, 
                    sensor_node_data[order].voltage, 
                    sensor_node_data[order].current, 
                    sensor_node_data[order].lumiorsity_value,
                    sensor_node_data[order].soil_moisture);
        // Gửi giá trị lên ERA IoT
        uint8_t virtualPin = 8 * order + data[index];
        ERa.virtualWrite(virtualPin, sensorValueFloat);
        Serial.printf("Gửi giá trị %f của cảm biến %02X lên ERA chân V%d\n", sensorValueFloat, data[index], virtualPin);
        index += 3;
    }
    Serial.println("Data sent to ERA successfully.");
}

// hàm này sẽ cập nhật trạng thái của các cảm biến lên màn hình TFT
void handleSensorConfig(uint8_t *data, uint8_t length)
{
    // kiểm tra địa chỉ mac để lấy thứ tự cảm biến
    uint8_t mac[NUMBER_OF_MAC_BYTES];
    for (int i = 0; i < NUMBER_OF_MAC_BYTES; i++)
    {
        mac[i] = data[4 + i]; // lấy địa chỉ mac
    }
    uint8_t order = 0;
    for (int i = 0; i < NUMBER_OF_SENS_NODES; i++)
    {
        if (memcmp(mac, mac_sens_node[i], NUMBER_OF_MAC_BYTES) == 0)
        {
            order = i + 1;
            break;
        }
    }
    if (order == 0)
    {
        Serial.println("Invalid MAC address!");
        return;
    }
    Serial.println("📦 SENSOR CONFIG được phát hiện, gọi decode...");
    uint8_t dataLength = length - NUMBER_OF_FIXED_BYTES;
    if (dataLength < 16)
    {
        Serial.println("❌ SENSOR CONFIG packet quá ngắn!");
        return;
    }

    sensor_state sensor_status;
    memset(&sensor_status, 0, sizeof(sensor_status));

    // gửi ACK
    vTaskDelay(pdMS_TO_TICKS(200));
    Serial.println("Sending ACK...");
    sendNodePacket(ACK, mac);
    // delay(5); // để đảm bảo gửi xong
    // yield();  // nhả CPU
    Serial.print("ACK sent to MAC: ");
    for (int i = 0; i < NUMBER_OF_MAC_BYTES; i++)
    {
        Serial.printf("%02X", mac[i]);
        if (i < 5)
            Serial.print(":");
    }
    Serial.println();

    // Vị trí bắt đầu của payload
    uint8_t payloadIndex = 11;

    for (int i = 0; i < 8; i++)
    {
        uint8_t sensorType = data[payloadIndex++];
        uint8_t sensorState = data[payloadIndex++];

        bool isActive = (sensorState == 1);

        switch (sensorType)
        {
        case VOLTAGE_SENSOR:
            sensor_status.voltage_sensor = isActive;
            break;
        case CURRENT_SENSOR:
            sensor_status.current_sensor = isActive;
            break;
        case LIGHT_SENSOR:
            sensor_status.light_sensor = isActive;
            break;
        case TEMPERATURE_SENSOR:
            sensor_status.temperature_sensor = isActive;
            break;
        case HUMIDITY_SENSOR:
            sensor_status.humidity_sensor = isActive;
            break;
        case SOIL_MOISTURE_SENSOR:
            sensor_status.soil_moisture_sensor = isActive;
            break;
        case EC_SENSOR:
            sensor_status.ec_sensor = isActive;
            break;
        case PH_SENSOR:
            sensor_status.ph_sensor = isActive;
            break;
        default:
            Serial.printf("⚠️ Sensor Type không xác định: %d\n", sensorType);
            break;
        }
    }

    // In trạng thái cảm biến
    Serial.println("✅ Đã giải mã SENSOR CONFIG:");
    Serial.printf("  Voltage Sensor: %s\n", sensor_status.voltage_sensor ? "Active" : "Inactive");
    Serial.printf("  Current Sensor: %s\n", sensor_status.current_sensor ? "Active" : "Inactive");
    Serial.printf("  Light Sensor: %s\n", sensor_status.light_sensor ? "Active" : "Inactive");
    Serial.printf("  Temperature Sensor: %s\n", sensor_status.temperature_sensor ? "Active" : "Inactive");
    Serial.printf("  Humidity Sensor: %s\n", sensor_status.humidity_sensor ? "Active" : "Inactive");
    Serial.printf("  Soil Moisture Sensor: %s\n", sensor_status.soil_moisture_sensor ? "Active" : "Inactive");
    Serial.printf("  EC Sensor: %s\n", sensor_status.ec_sensor ? "Active" : "Inactive");
    Serial.printf("  pH Sensor: %s\n", sensor_status.ph_sensor ? "Active" : "Inactive");
}

void handleControlPacket(uint8_t *data, uint8_t length, uint8_t packetType)
{
    // kiểm tra địa chỉ mac để lấy thứ tự node điều khiển
    uint8_t mac[NUMBER_OF_MAC_BYTES];
    for (int i = 0; i < NUMBER_OF_MAC_BYTES; i++)
    {
        mac[i] = data[4 + i]; // lấy địa chỉ mac
    }
    uint8_t order = 0;
    // printMacAddress(mac);
    for (int i = 0; i < NUMBER_OF_SENS_NODES; i++)
    {
        printMacAddress(mac_ctrl_node[i]);
        if (memcmp(mac, mac_ctrl_node[i], NUMBER_OF_MAC_BYTES) == 0)
        {
            order = i + 1;
            break;
        }
    }
    if (order == 0)
    {
        Serial.println("Invalid MAC address!");
        return;
    }
    uint8_t payloadLength = length - NUMBER_OF_FIXED_BYTES;
    if (packetType == STATUS)
    {
        Serial.println("Received STATUS from node:");
        Serial.print("Payload: ");
        for (int i = 0; i < payloadLength; i++)
        {
            // Serial.print((char)data[i + 11]);
            Serial.printf("%02X ", data[i + 11]);
        }
        Serial.println();
        vTaskDelay(pdMS_TO_TICKS(100));
        ; // để đảm bảo gửi xong
        Serial.println("Sending ACK...");
        sendNodePacket(ACK, mac);
        // delay(5); // để đảm bảo gửi xong
        // yield();  // nhả CPU
        Serial.print("ACK sent to MAC: ");
        for (int i = 0; i < NUMBER_OF_MAC_BYTES; i++)
        {
            Serial.printf("%02X", mac[i]);
            if (i < 5)
                Serial.print(":");
        }
        Serial.println();
    }
    else if (packetType == CONFIG_REQ)
    {
        Serial.println("Received CONFIG_REQ from node:");
        Serial.print("Payload: ");
        for (int i = 0; i < payloadLength; i++)
        {
            Serial.print((char)data[i]);
        }
        Serial.println();
        vTaskDelay(pdMS_TO_TICKS(100));
        ; // để đảm bảo gửi xong
        sendControlConfig(mac);
    }
}

void sendControlConfig(const uint8_t *mac)
{
    // buffer sẽ có 14 bytes cố định, thêm 3 * NUMBER_OF_RELAYS bytes cho các relay (relay ID, value, duration)
    uint8_t bufferLength = NUMBER_OF_FIXED_BYTES + 3 * NUMBER_OF_RELAYS;

    uint8_t buffer[bufferLength];
    uint8_t i = 0;

    // STX
    buffer[i++] = STX1;
    buffer[i++] = STX2;
    // LENGTH
    buffer[i++] = (bufferLength >> 8) & 0xFF;
    buffer[i++] = bufferLength & 0xFF;
    // MAC đích
    for (int j = 0; j < NUMBER_OF_MAC_BYTES; j++)
        buffer[i++] = mac[j];

    // TYPE = CONFIG
    buffer[i++] = CONFIG_CTRL;

    // PAYLOAD: relay_id, value, duration
    for (uint8_t r = 0; r < NUMBER_OF_RELAYS; r++)
    {
        buffer[i++] = r + 1; // relay ID
        buffer[i++] = 0x00;  // value = OFF
        buffer[i++] = 0x00;  // duration = 0
    }

    // END
    buffer[i++] = END_BYTE;

    // CRC16
    uint16_t crc = CalculateCRC(buffer, bufferLength - 2);
    buffer[i++] = (crc >> 8) & 0xFF;
    buffer[i++] = crc & 0xFF;

    // In các byte của buffer qua Serial trước khi gửi
    Serial.print("Sending CONFIG Packet: ");
    for (int j = 0; j < bufferLength; j++)
    {
        Serial.printf("%02X ", buffer[j]);
    }
    Serial.println();

    // Gửi và chuyển sang chế độ nhận để đảm bảo không bị treo
    if (LoRa.beginPacket())
    {
        LoRa.write(buffer, bufferLength);
        LoRa.endPacket();
        // LoRa.receive(); // Chuyển sang chế độ nhận sau khi gửi lệnh
        LoRa.idle();
    }
}

void sendControlCommand(const uint8_t *mac, uint8_t relay, uint8_t value, uint8_t duration)
{
    // buffer sẽ có 14 bytes cố định, thêm 3 bytes cho khung dữ liệu - relay, value, duration
    uint8_t length = NUMBER_OF_FIXED_BYTES + 3; // Tính độ dài khung dữ liệu
    uint8_t buffer[length];
    uint8_t i = 0;
    memset(buffer, 0x00, sizeof(buffer));
    // STX
    buffer[i++] = STX1;
    buffer[i++] = STX2;
    // LENGTH
    buffer[i++] = (length >> 8) & 0xFF;
    buffer[i++] = length & 0xFF;

    // MAC đích
    for (int j = 0; j < NUMBER_OF_MAC_BYTES; j++)
    {
        buffer[i++] = mac[j];
    }

    // TYPE
    buffer[i++] = CONTROL_CMD;

    // PAYLOAD
    buffer[i++] = relay;
    buffer[i++] = value;
    buffer[i++] = duration;

    // END identification
    buffer[i++] = END_BYTE;

    // CRC16 cho toàn bộ frame trừ STX
    uint16_t crc = CalculateCRC(buffer, length - 2);
    buffer[i++] = (crc >> 8) & 0xFF;
    buffer[i++] = crc & 0xFF;

    // Kiểm tra trạng thái LoRa
    if (LoRa.beginPacket())
    {
        Serial.print("Sent packet: ");
        for (int i = 0; i < length; i++)
        {
            Serial.print(buffer[i], HEX);
            Serial.print(" ");
        }
        LoRa.write(buffer, length);
        LoRa.endPacket();
        // LoRa.receive(); // Chuyển sang chế độ nhận sau khi gửi lệnh
        LoRa.idle();
        // in ra packet đã gửi
        Serial.println("LoRa command sent successfully.");
    }
    else
    {
        Serial.println("LoRa not ready, failed to send command.");
    }
}

void sendNodePacket(uint8_t packetType, const uint8_t *mac)
{
    // buffer sẽ có 14 bytes cố định, thêm 1 hoặc 2 bytes cho dữ liệu
    uint8_t length = NUMBER_OF_FIXED_BYTES;
    if (packetType == ACK)
    {
        length += 2; // 2 bytes cho mỗi relay
    }
    else if (packetType == CONFIG_REQ)
    {
        length += 1; // 1 byte cho lệnh cấu hình
    }
    else
        Serial.println("Invalid packet type!");
    uint8_t buf[length];
    int index = 0;
    memset(buf, 0x00, sizeof(buf));

    // bắt đầu gói tin
    buf[index++] = STX1; // 0x4E
    buf[index++] = STX2; // 0x57
    buf[index++] = (length >> 8) & 0xFF;
    buf[index++] = length & 0xFF;
    for (int i = 0; i < NUMBER_OF_MAC_BYTES; i++)
        buf[index++] = mac[i];
    buf[index++] = packetType;

    if (packetType == ACK)
    {
        buf[index++] = 'O';
        buf[index++] = 'K';
    }
    else if (packetType == CONFIG_REQ)
    {
        buf[index++] = 0x00;
    }

    buf[index++] = END_BYTE; // 0x68

    uint16_t crc = CalculateCRC(buf, length - 2);
    buf[index++] = (crc >> 8) & 0xFF;
    buf[index++] = crc & 0xFF;

    Serial.print("✅ Sending ACK/ CONFIG_REQ Packet: ");
    for (int i = 0; i < index; i++)
    {
        Serial.printf("%02X ", buf[i]);
    }
    Serial.println();

    LoRa.beginPacket();
    LoRa.write(buf, length);
    LoRa.endPacket();
    // delay(5);
    LoRa.receive(); // Chuyển sang chế độ nhận sau khi gửi lệnh
    // LoRa.idle();
}

#endif

#if defined(CONTROL_NODE)
volatile bool relayState[NUMBER_OF_RELAYS] = {false, false, false, false, false};
bool relayAutoStart[NUMBER_OF_RELAYS] = {false, false, false, false, false};
unsigned long relayStartTime[NUMBER_OF_RELAYS] = {0, 0, 0, 0, 0};
uint8_t relayDurationControl[NUMBER_OF_RELAYS] = {0, 0, 0, 0, 0};

// Hàm xử lý ngắt
void IRAM_ATTR handleBtn1()
{
    toggleRelay(1);
}

void IRAM_ATTR handleBtn2()
{
    toggleRelay(2);
}
void IRAM_ATTR handleBtn3()
{
    toggleRelay(3);
}

void IRAM_ATTR handleBtn4()
{
    toggleRelay(4);
}

void IRAM_ATTR handleBtn5()
{
    toggleRelay(5);
}

void initRelay(void)
{
    for (int i = 0; i < NUMBER_OF_RELAYS; i++)
    {
        pinMode(relayPins[i], OUTPUT);
        digitalWrite(relayPins[i], HIGH);
    }
}

void initButton(void)
{
    for (int i = 0; i < NUMBER_OF_RELAYS; i++)
    {
        pinMode(buttonPins[i], INPUT);
    }

    // Gán ngắt
    attachInterrupt(digitalPinToInterrupt(buttonPins[0]), handleBtn1, FALLING);
    attachInterrupt(digitalPinToInterrupt(buttonPins[1]), handleBtn2, FALLING);
    attachInterrupt(digitalPinToInterrupt(buttonPins[2]), handleBtn3, FALLING);
    attachInterrupt(digitalPinToInterrupt(buttonPins[3]), handleBtn4, FALLING);
    attachInterrupt(digitalPinToInterrupt(buttonPins[4]), handleBtn5, FALLING);
}

void setRelay(uint8_t id, bool state)
{
    digitalWrite(relayPins[id - 1], state ? LOW : HIGH);
    relayState[id - 1] = state;
}

void toggleRelay(uint8_t id)
{
    relayState[id - 1] = !relayState[id - 1];
    digitalWrite(relayPins[id - 1], relayState[id - 1]);
}

void sendControlPacket(uint8_t packetType, const uint8_t *macAdress)
{
    // buffer sẽ có 14 bytes cố định, thêm 10 bytes cho dữ liệu trạng thái
    uint8_t length = NUMBER_OF_FIXED_BYTES;
    if (packetType == STATUS)
    {
        length += 2 * 5; // 2 bytes cho mỗi relay
    }
    else if (packetType == CONFIG_REQ)
    {
        length += 1; // 1 byte cho lệnh cấu hình
    }
    else
        Serial.println("Invalid packet type!");

    uint8_t buf[length];
    int index = 0;
    memset(buf, 0x00, sizeof(buf)); // Reset buffer (gán tất cả phần tử trong buffer về 0x00 hoặc 0xFF)

    // bắt đầu gói tin
    buf[index++] = STX1;
    buf[index++] = STX2;
    buf[index++] = length >> 8;
    buf[index++] = length & 0xFF;
    for (int i = 0; i < NUMBER_OF_MAC_BYTES; i++)
        buf[index++] = macAdress[i];
    buf[index++] = packetType;

    if (packetType == STATUS)
    {
        buf[index++] = RELAY_1_HEADER;
        buf[index++] = relayState[0] ? 0x01 : 0x00;
        buf[index++] = RELAY_2_HEADER;
        buf[index++] = relayState[1] ? 0x01 : 0x00;
        buf[index++] = RELAY_3_HEADER;
        buf[index++] = relayState[2] ? 0x01 : 0x00;
        buf[index++] = RELAY_4_HEADER;
        buf[index++] = relayState[3] ? 0x01 : 0x00;
        buf[index++] = RELAY_5_HEADER;
        buf[index++] = relayState[4] ? 0x01 : 0x00;
    }
    else if (packetType == CONFIG_REQ)
    {
        buf[index++] = 0x00;
    }

    buf[index++] = END_BYTE;
    // 2 bytes cuối là checksum, sử dụng CRC16
    uint16_t crc = CalculateCRC(buf, length - 2); // chuẩn hóa như bên Gateway
    buf[index++] = (crc >> 8) & 0xFF;
    buf[index++] = crc & 0xFF;

    Serial.print("🔼 Sent STATUS/CONFIG Packet: ");
    for (int i = 0; i < index; i++)
    {
        Serial.printf("%02X ", buf[i]);
    }
    Serial.println();

    LoRa.beginPacket();      // Bắt đầu gói tin
    LoRa.write(buf, length); // Ghi dữ liệu vào gói tin
    LoRa.endPacket();        // Kết thúc gói tin
    // LoRa.receive();          // <- Cái này cũng nên thêm
    LoRa.idle();
}

void decodeControlPacket(uint8_t *data, uint8_t incomingLength, const uint8_t *macAdress)
{
    // Kiểm tra header
    if (data[0] != STX1 || data[1] != STX2)
    {
        Serial.println("Invalid packet header!");
        return;
    }

    // Kiểm tra độ dài gói tin
    uint16_t length = (data[2] << 8) | data[3];
    if (length != incomingLength)
    {
        Serial.println("Invalid packet length!");
        return;
    }

    // Kiểm tra CRC
    uint16_t crc = CalculateCRC(data, length - 2);
    if (crc != ((data[length - 2] << 8) | data[length - 1]))
    {
        Serial.println(crc, HEX);
        Serial.println("Invalid CRC!");
        return;
    }

    // kiểm tra end byte (0x68)
    if (data[length - 3] != END_BYTE)
    {
        Serial.println("Invalid end byte!");
        return;
    }

    // So sánh MAC
    for (int i = 0; i < NUMBER_OF_MAC_BYTES; i++)
    {
        if (data[4 + i] != macAdress[i])
        {
            Serial.println("MAC addresses are not equal.");
            return;
        }
    }
    // cấu hình lệnh
    uint8_t packetType = data[10];
    // xử lý địa chỉ mac ở trong các hàm dưới đây
    switch (packetType)
    {
    case CONTROL_CMD:
        Serial.println("Received control command");
        handleControlCommand(data, macAdress);
        break;
    case CONFIG_CTRL:
        Serial.println("Received config command");
        handleControlConfig(data, length);
        break;
    default:
        Serial.printf("⚠️ Package Type không xác định: %d\n", packetType);
        break;
    }
}

void handleControlCommand(uint8_t *data, const uint8_t *macAdress)
{
    uint8_t ID = data[11];
    uint8_t Value = data[12];
    uint8_t Duration = data[13];

    // Serial.printf("Relay %d: %s (%ds)\n", relayID, relayValue ? "ON" : "OFF", relayDurationControl);

    if (Value == 1)
    {
        setRelay(ID, true);
        unsigned long now = millis();
        // nêu relayDurationControl > 0 thì là chế độ tự động, tắt sau thời gian này
        // còn lại thì là chế độ thủ công, chờ lệnh tắt từ gateway
        if (Duration > 0)
        {
            relayAutoStart[ID - 1] = true;
            relayStartTime[ID - 1] = now;
            relayDurationControl[ID - 1] = Duration;
            Serial.printf("Relay %d AUTO ON (%ds)\n", ID, Duration);
        }
        else
        {
            relayAutoStart[ID - 1] = false;
            Serial.printf("Relay %d MANUAL ON\n", ID);
        }
    }
    else
    {
        setRelay(ID, false);
        relayAutoStart[ID - 1] = false;
        Serial.printf("Relay %d MANUAL OFF\n", ID);
    }
    Serial.println("🔼 Sending STATUS: ");
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay 1s
    // delay(500); // không gửi quá nhiều gói tin liên tiếp, gateway không xử lý kịp
    sendControlPacket(STATUS, macAdress);
    Serial.println("🕒 Chờ ACK");
    if (!waitForAck(1000, macAdress))
    {
        Serial.println("❌ ACK timeout");
    }
    else
    {
        Serial.println("✅ ACK received!");
    }
    // LoRa.receive(); // đảm bảo quay lại RX mode
    LoRa.idle();
}

void handleControlConfig(uint8_t *data, uint8_t length)
{
    // Vị trí bắt đầu của payload
    uint8_t payloadIndex = 11;
    uint8_t payloadLength = 3 * NUMBER_OF_RELAYS;
    for (int i = payloadIndex; i < payloadIndex + payloadLength; i += 3)
    {
        uint8_t ID = data[i];
        uint8_t Value = data[i + 1];
        uint8_t Duration = data[i + 2];
        setRelay(ID, Value);
    }
}

void checkAutoOffRelays()
{
    // Kiểm tra tự OFF theo millis
    unsigned long now = millis();
    for (int i = 0; i < NUMBER_OF_RELAYS; i++)
    {
        if (relayAutoStart[i] && relayState[i] && (now - relayStartTime[i] >= 1000UL * relayDurationControl[i]))
        {
            setRelay(i + 1, false);
            relayAutoStart[i] = false;
            Serial.printf("Relay %d AUTO OFF\n", i + 1);
        }
    }
}
#endif

void getMacAddress(uint8_t *mac)
{
    String macStr = WiFi.macAddress(); // VD: "24:6F:28:3A:BC:D4"
    int i = 0;
    char *token = strtok((char *)macStr.c_str(), ":");
    while (token != NULL && i < NUMBER_OF_MAC_BYTES)
    {
        mac[i++] = strtol(token, NULL, 16);
        token = strtok(NULL, ":");
    }
}

void printMacAddress(const uint8_t *mac)
{
    for (int i = 0; i < NUMBER_OF_MAC_BYTES; i++)
    {
        Serial.printf("%02X", mac[i]);
        if (i < 5)
            Serial.print(":");
    }
    Serial.println();
}

bool compareMacAddress(const uint8_t *mac1, const uint8_t *mac2)
{
    // Serial.print("Comparing MAC addresses: ");
    // Serial.print("MAC1: ");
    // printMacAddress(mac1);
    // Serial.print("MAC2: ");
    // printMacAddress(mac2);

    if (memcmp(mac1, mac2, 6) == 0)
    {
        Serial.println("MAC addresses are equal.");
        return true;
    }
    else
    {
        Serial.println("MAC addresses are not equal.");
        return false;
    }
}

bool waitForAck(uint32_t timeout, const uint8_t *expectedMac)
{
    unsigned long startTime = millis();

    while (millis() - startTime < timeout)
    {
        int len = LoRa.parsePacket();
        if (len > 0)
        {
            uint8_t buf[64];
            int i = 0;
            while (LoRa.available() && i < sizeof(buf))
            {
                buf[i++] = LoRa.read();
            }

            Serial.print("📥 RAW PACKET: ");
            for (int j = 0; j < i; j++)
                Serial.printf("%02X ", buf[j]);
            Serial.println();

            uint16_t length = (buf[2] << 8) | buf[3];
            if (i < length)
            {
                Serial.printf("❌ Length mismatch: frameLen=%d, actual=%d\n", length, i);
                break;
                ;
            }
            if (buf[0] != STX1 || buf[1] != STX2)
            {
                Serial.println("❌ Invalid header");
                break;
            }
            if (buf[length - 3] != END_BYTE) // END_BYTE
            {
                Serial.println("❌ Invalid end byte");
                break;
            }

            uint16_t recvCRC = (buf[length - 2] << 8) | buf[length - 1];
            uint16_t calcCRC = CalculateCRC(buf, length - 2); // Tính từ LENGTH đến END_BYTE
            if (recvCRC != calcCRC)
            {
                Serial.println("❌ Invalid CRC");
                break;
            }

            if (buf[10] != ACK)
            {
                Serial.println("❌ Expected ACK, got something else");
                break;
            }

            if (buf[11] != 'O' || buf[12] != 'K')
            {
                Serial.println("❌ Invalid message, not OK");
                break;
            }

            bool macMatch = true;
            for (int j = 0; j < NUMBER_OF_MAC_BYTES; j++)
            {
                if (buf[4 + j] != expectedMac[j])
                {
                    macMatch = false;
                    break;
                }
            }

            if (!macMatch)
            {
                Serial.println("❌ MAC mismatch");
                break;
            }

            // Serial.println("✅ ACK received!");
            return true;
        }
    }
    return false;
}

uint16_t CalculateCRC(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* TFT screen*/
#if defined(GATEWAY_NODE)
TFT_eSPI tft = TFT_eSPI(); // Invoke custom library
bool touch_test = false;
tft_screen_select_t tft_screen_select = MAIN_DASHBOARD; // Default screen
uint8_t current_sensor_node_id = 0, current_control_node_id = 0;

void init_TFT_screen()
{
    // Initialise the TFT screen
    tft.init();

    // Set the rotation to the orientation you wish to use in your project before calibration
    // (the touch coordinates returned then correspond to that rotation only)
    tft.setRotation(1);

    // Calibrate the touch screen and retrieve the scaling factors
    // touch_calibrate();

    // Replace above line with the code sent to Serial Monitor
    // once calibration is complete, e.g.:
    uint16_t calData[5] = {343, 3352, 524, 2981, 1};
    tft.setTouch(calData);

    // Clear the screen
    tft.fillScreen(TFT_BLACK);
}

void test_touch_screen()
{
    if (!touch_test)
    {
        // Clear the screen
        tft.fillScreen(TFT_BLACK);
        tft.drawCentreString("Touch screen to test!", tft.width() / 2, tft.height() / 2, 2);
        touch_test = true;
    }

    uint16_t x = 0, y = 0; // To store the touch coordinates

    // Pressed will be set true is there is a valid touch on the screen
    bool pressed = tft.getTouch(&x, &y);

    // Draw a white spot at the detected coordinates
    if (pressed)
    {
        tft.fillCircle(x, y, 2, TFT_WHITE);
        // Serial.print("x,y = ");
        // Serial.print(x);
        // Serial.print(",");
        // Serial.println(y);
    }
}

unsigned long lastTouchTime = 0;
const unsigned long debounceDelay = 200; // Thời gian tối thiểu giữa 2 lần nhấn (ms)

void check_touch_screen()
{
    uint16_t x = 0, y = 0; // To store the touch coordinates
    // Pressed will be set true is there is a valid touch on the screen
    bool pressed = tft.getTouch(&x, &y);

    if (pressed)
    {
        unsigned long now = millis();
        if (now - lastTouchTime > debounceDelay)
        {
            lastTouchTime = now;
            switch (tft_screen_select)
            {
            case MAIN_DASHBOARD:
                if (x >= 20 && x <= 150 && y >= 210 && y <= 230)
                {
                    tft_screen_select = NODE_DASHBOARD;
                    draw_node_dashboard();
                }
                break;

            case NODE_DASHBOARD:
                if (x >= 0 && x <= 15 && y >= 0 && y <= 15)
                {
                    tft_screen_select = MAIN_DASHBOARD;
                    draw_main_dashboard();
                    update_main_dashboard_time();
                    update_main_dashboard_sensor();
                    update_main_dashboard_communicate();
                }
                else if (x >= 20 && x <= 50 && y >= 126 && y <= 141)
                {
                    current_sensor_node_id = 1;
                    tft_screen_select = SENSOR_DASHBOARD;
                    draw_sensor_dashboard(current_sensor_node_id);
                }
                else if (x >= 17 && x <= 47 && y >= 204 && y <= 219)
                {
                    current_sensor_node_id = 2;
                    tft_screen_select = SENSOR_DASHBOARD;
                    draw_sensor_dashboard(current_sensor_node_id);
                }
                else if (x >= 135 && x <= 165 && y >= 185 && y <= 200)
                {
                    current_sensor_node_id = 3;
                    tft_screen_select = SENSOR_DASHBOARD;
                    draw_sensor_dashboard(current_sensor_node_id);
                }
                else if (x >= 269 && x <= 299 && y >= 146 && y <= 161)
                {
                    current_control_node_id = 1;
                    tft_screen_select = CONTROL_DASHBOARD;
                    draw_control_dashboard(current_control_node_id);
                }
                else if (x >= 269 && x <= 299 && y >= 213 && y <= 228)
                {
                    current_control_node_id = 2;
                    tft_screen_select = CONTROL_DASHBOARD;
                    draw_control_dashboard(current_control_node_id);
                }
                break;

            case SENSOR_DASHBOARD:
                if (x >= 0 && x <= 15 && y >= 0 && y <= 15)
                {
                    tft_screen_select = NODE_DASHBOARD;
                    draw_node_dashboard();
                }
                if (x >= 96 && x <= 220 && y >= 210 && y <= 236)
                {
                    sendNodePacket(CONFIG_REQ, mac_sens_node[current_sensor_node_id - 1]);
                }
                break;

            case CONTROL_DASHBOARD:
                if (x >= 0 && x <= 15 && y >= 0 && y <= 15)
                {
                    tft_screen_select = NODE_DASHBOARD;
                    draw_node_dashboard();
                }
                // Relay 1
                else if (x >= 27 && x <= 67 && y >= 53 && y <= 93)
                {
                    RELAY_1[current_control_node_id - 1] = !RELAY_1[current_control_node_id - 1];
                    sendControlCommand(mac_ctrl_node[current_control_node_id - 1], 1, uint8_t(RELAY_1[current_control_node_id - 1]), 0);
                    update_control_dashboard(current_control_node_id);
                }
                else if (x >= 141 && x <= 181 && y >= 53 && y <= 93)
                {
                    RELAY_2[current_control_node_id - 1] = !RELAY_2[current_control_node_id - 1];
                    sendControlCommand(mac_ctrl_node[current_control_node_id - 1], 2, uint8_t(RELAY_2[current_control_node_id - 1]), 0);
                    update_control_dashboard(current_control_node_id);
                }
                else if (x >= 252 && x <= 292 && y >= 53 && y <= 93)
                {
                    RELAY_3[current_control_node_id - 1] = !RELAY_3[current_control_node_id - 1];
                    sendControlCommand(mac_ctrl_node[current_control_node_id - 1], 3, uint8_t(RELAY_3[current_control_node_id - 1]), 0);
                    update_control_dashboard(current_control_node_id);
                }
                else if (x >= 84 && x <= 124 && y >= 143 && y <= 163)
                {
                    RELAY_4[current_control_node_id - 1] = !RELAY_4[current_control_node_id - 1];
                    sendControlCommand(mac_ctrl_node[current_control_node_id - 1], 4, uint8_t(RELAY_4[current_control_node_id - 1]), 0);
                    update_control_dashboard(current_control_node_id);
                }
                else if (x >= 95 && x <= 235 && y >= 143 && y <= 163)
                {
                    RELAY_5[current_control_node_id - 1] = !RELAY_5[current_control_node_id - 1];
                    sendControlCommand(mac_ctrl_node[current_control_node_id - 1], 5, uint8_t(RELAY_5[current_control_node_id - 1]), 0);
                    update_control_dashboard(current_control_node_id);
                }
                break;

            default:
                break;
            }
        }
    }
}

void touch_calibrate()
{
    uint16_t calData[5];
    uint8_t calDataOK = 0;

    // Calibrate
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.println("Touch corners as indicated");

    tft.setTextFont(1);
    tft.println();

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

    Serial.println();
    Serial.println();
    Serial.println("// Use this calibration code in setup():");
    Serial.print("  uint16_t calData[5] = ");
    Serial.print("{ ");

    for (uint8_t i = 0; i < 5; i++)
    {
        Serial.print(calData[i]);
        if (i < 4)
            Serial.print(", ");
    }

    Serial.println(" };");
    Serial.print("  tft.setTouch(calData);");
    Serial.println();
    Serial.println();

    tft.fillScreen(TFT_BLACK);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");
    tft.println("Calibration code sent to Serial port.");

    delay(4000);
}

// ####################################################################################################
//  Draw a JPEG on the TFT pulled from SD Card
// ####################################################################################################
//  xpos, ypos is top left corner of plotted image
void drawSdJpeg(const char *filename, int xpos, int ypos)
{

    // Open the named file (the Jpeg decoder library will close it)
    File jpegFile = SD.open(filename, FILE_READ); // or, file handle reference for SD library

    if (!jpegFile)
    {
        Serial.print("ERROR: File \"");
        Serial.print(filename);
        Serial.println("\" not found!");
        return;
    }

    Serial.println("===========================");
    Serial.print("Drawing file: ");
    Serial.println(filename);
    Serial.println("===========================");

    // Use one of the following methods to initialise the decoder:
    bool decoded = JpegDec.decodeSdFile(jpegFile); // Pass the SD file handle to the decoder,
    // bool decoded = JpegDec.decodeSdFile(filename);  // or pass the filename (String or character array)

    if (decoded)
    {
        // print information about the image to the serial port
        //   jpegInfo();
        // render the image onto the screen at given coordinates
        jpegRender(xpos, ypos);
    }
    else
    {
        Serial.println("Jpeg file format not supported!");
    }
}

// ####################################################################################################
//  Draw a JPEG on the TFT, images will be cropped on the right/bottom sides if they do not fit
// ####################################################################################################
//  This function assumes xpos,ypos is a valid screen coordinate. For convenience images that do not
//  fit totally on the screen are cropped to the nearest MCU size and may leave right/bottom borders.
void jpegRender(int xpos, int ypos)
{

    // jpegInfo(); // Print information from the JPEG file (could comment this line out)

    uint16_t *pImg;
    uint16_t mcu_w = JpegDec.MCUWidth;
    uint16_t mcu_h = JpegDec.MCUHeight;
    uint32_t max_x = JpegDec.width;
    uint32_t max_y = JpegDec.height;

    bool swapBytes = tft.getSwapBytes();
    tft.setSwapBytes(true);

    // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
    // Typically these MCUs are 16x16 pixel blocks
    // Determine the width and height of the right and bottom edge image blocks
    uint32_t min_w = jpg_min(mcu_w, max_x % mcu_w);
    uint32_t min_h = jpg_min(mcu_h, max_y % mcu_h);

    // save the current image block size
    uint32_t win_w = mcu_w;
    uint32_t win_h = mcu_h;

    // record the current time so we can measure how long it takes to draw an image
    uint32_t drawTime = millis();

    // save the coordinate of the right and bottom edges to assist image cropping
    // to the screen size
    max_x += xpos;
    max_y += ypos;

    // Fetch data from the file, decode and display
    while (JpegDec.read())
    {                          // While there is more data in the file
        pImg = JpegDec.pImage; // Decode a MCU (Minimum Coding Unit, typically a 8x8 or 16x16 pixel block)

        // Calculate coordinates of top left corner of current MCU
        int mcu_x = JpegDec.MCUx * mcu_w + xpos;
        int mcu_y = JpegDec.MCUy * mcu_h + ypos;

        // check if the image block size needs to be changed for the right edge
        if (mcu_x + mcu_w <= max_x)
            win_w = mcu_w;
        else
            win_w = min_w;

        // check if the image block size needs to be changed for the bottom edge
        if (mcu_y + mcu_h <= max_y)
            win_h = mcu_h;
        else
            win_h = min_h;

        // copy pixels into a contiguous block
        if (win_w != mcu_w)
        {
            uint16_t *cImg;
            int p = 0;
            cImg = pImg + win_w;
            for (int h = 1; h < win_h; h++)
            {
                p += mcu_w;
                for (int w = 0; w < win_w; w++)
                {
                    *cImg = *(pImg + w + p);
                    cImg++;
                }
            }
        }

        // calculate how many pixels must be drawn
        uint32_t mcu_pixels = win_w * win_h;

        // draw image MCU block only if it will fit on the screen
        if ((mcu_x + win_w) <= tft.width() && (mcu_y + win_h) <= tft.height())
            tft.pushImage(mcu_x, mcu_y, win_w, win_h, pImg);
        else if ((mcu_y + win_h) >= tft.height())
            JpegDec.abort(); // Image has run off bottom of screen so abort decoding
    }

    tft.setSwapBytes(swapBytes);

    showTime(millis() - drawTime); // These lines are for sketch testing only
}

// ####################################################################################################
//  Print image information to the serial port (optional)
// ####################################################################################################
//  JpegDec.decodeFile(...) or JpegDec.decodeArray(...) must be called before this info is available!
void jpegInfo()
{

    // Print information extracted from the JPEG file
    Serial.println("JPEG image info");
    Serial.println("===============");
    Serial.print("Width      :");
    Serial.println(JpegDec.width);
    Serial.print("Height     :");
    Serial.println(JpegDec.height);
    Serial.print("Components :");
    Serial.println(JpegDec.comps);
    Serial.print("MCU / row  :");
    Serial.println(JpegDec.MCUSPerRow);
    Serial.print("MCU / col  :");
    Serial.println(JpegDec.MCUSPerCol);
    Serial.print("Scan type  :");
    Serial.println(JpegDec.scanType);
    Serial.print("MCU width  :");
    Serial.println(JpegDec.MCUWidth);
    Serial.print("MCU height :");
    Serial.println(JpegDec.MCUHeight);
    Serial.println("===============");
    Serial.println("");
}

// ####################################################################################################
//  Show the execution time (optional)
// ####################################################################################################
//  WARNING: for UNO/AVR legacy reasons printing text to the screen with the Mega might not work for
//  sketch sizes greater than ~70KBytes because 16-bit address pointers are used in some libraries.

// The Due will work fine with the HX8357_Due library.

void showTime(uint32_t msTime)
{
    // tft.setCursor(0, 0);
    // tft.setTextFont(1);
    // tft.setTextSize(2);
    // tft.setTextColor(TFT_WHITE, TFT_BLACK);
    // tft.print(F(" JPEG drawn in "));
    // tft.print(msTime);
    // tft.println(F(" ms "));
    Serial.print(F(" JPEG drawn in "));
    Serial.print(msTime);
    Serial.println(F(" ms "));
}

#if defined(TFT_BMP)
// Hàm lấy từ ví dụ gốc của thư viện
void drawBmp(const char *filename, int16_t x, int16_t y)
{
    tft.setRotation(1); // landscape
    tft.fillScreen(random(0xFFFF));

    File bmpFile;
    int bmpWidth, bmpHeight;
    uint8_t bmpDepth;
    uint32_t bmpImageoffset;
    uint32_t rowSize;
    uint8_t sdbuffer[3 * 20]; // 20 pixel buffer
    uint8_t buffidx = sizeof(sdbuffer);
    boolean goodBmp = false;

    if ((x >= tft.width()) || (y >= tft.height()))
        return;

    bmpFile = SD.open(filename);
    if (!bmpFile)
    {
        Serial.print("File không tìm thấy: ");
        Serial.println(filename);
        return;
    }

    if (read16(bmpFile) == 0x4D42)
    {                                     // BMP signature
        (void)read32(bmpFile);            // fileSize
        (void)read32(bmpFile);            // creator bytes
        bmpImageoffset = read32(bmpFile); // Start of image data
        (void)read32(bmpFile);            // header size
        bmpWidth = read32(bmpFile);
        bmpHeight = read32(bmpFile);
        if (read16(bmpFile) == 1)
        {
            bmpDepth = read16(bmpFile);
            if ((bmpDepth == 24) && (read32(bmpFile) == 0))
            {
                goodBmp = true;
                rowSize = (bmpWidth * 3 + 3) & ~3;

                bmpFile.seek(bmpImageoffset);
                for (int row = 0; row < bmpHeight; row++)
                {
                    bmpFile.seek(bmpImageoffset + (bmpHeight - 1 - row) * rowSize);
                    for (int col = 0; col < bmpWidth; col++)
                    {
                        bmpFile.read(sdbuffer, 3);
                        uint16_t color = tft.color565(sdbuffer[2], sdbuffer[1], sdbuffer[0]);
                        tft.drawPixel(x + col, y + row, color);
                    }
                }
            }
        }
    }
    bmpFile.close();
}

uint16_t read16(File &f)
{
    uint16_t result;
    ((uint8_t *)&result)[0] = f.read();
    ((uint8_t *)&result)[1] = f.read();
    return result;
}

uint32_t read32(File &f)
{
    uint32_t result;
    ((uint8_t *)&result)[0] = f.read();
    ((uint8_t *)&result)[1] = f.read();
    ((uint8_t *)&result)[2] = f.read();
    ((uint8_t *)&result)[3] = f.read();
    return result;
}
#endif

void draw_main_dashboard()
{
    drawSdJpeg("/image/main_dashboard.jpg", 0, 0); // This draws a jpeg pulled off the SD Card
    tft.setTextColor(TFT_WHITE);
    tft.setFreeFont(&FreeSerifBold9pt7b);
    tft.drawString("To node dashboard", 23, 214);
}

void draw_node_dashboard()
{
    drawSdJpeg("/image/node_dashboard.jpg", 0, 0); // This draws a jpeg pulled off the SD Card
    tft.setTextFont(1);
    tft.setTextSize(1);
    tft.setTextColor(TFT_BLACK); // Chữ đen, nền trong suốt
    tft.drawString("Sens 1", 25, 131);
    tft.drawString("Sens 2", 22, 209);
    tft.drawString("Sens 3", 140, 190);
    tft.drawString("Ctrl 1", 274, 151);
    tft.drawString("Ctrl 2", 274, 218);
}

void draw_sensor_dashboard(uint8_t ID)
{
    drawSdJpeg("/image/sensor_dashboard.jpg", 0, 0); // This draws a jpeg pulled off the SD Card
    tft.setTextFont(1);
    tft.setTextSize(1);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.drawString(String(ID), 200, 181);
    if (ID <= NUMBER_OF_SENS_NODES)
    {
        tft.drawString("Active", 105, 180);
        tft.drawString("IDLE", 156, 110);
        // đưa dữ liệu lên nè
        tft.drawString(String(sensor_node_data[ID - 1].voltage) + " %", 49, 72);
        tft.drawString(String(sensor_node_data[ID - 1].current) + "mWh", 153, 72);
        tft.drawString(String(sensor_node_data[ID - 1].temperature) + " °C", 257, 72);
        tft.drawString(String(sensor_node_data[ID - 1].humidity) + " %", 49, 110);
        tft.drawString(String(sensor_node_data[ID - 1].ec_value), 257, 110);
        tft.drawString(String(sensor_node_data[ID - 1].ph_value), 49, 152);
        tft.drawString(String(sensor_node_data[ID - 1].soil_moisture) + " %", 153, 152);
        tft.drawString(String(sensor_node_data[ID - 1].lumiorsity_value) + "lux", 257, 152);
    }
}

void draw_control_dashboard(uint8_t ID)
{
    drawSdJpeg("/image/control_dashboard.jpg", 0, 0); // This draws a jpeg pulled off the SD Card
    tft.setTextFont(1);
    tft.setTextSize(1);
    tft.setTextColor(TFT_BLACK);
    tft.drawString(String(ID), 260, 218);
    update_control_dashboard(current_control_node_id);
}

void update_main_dashboard_time()
{
    // Serial.println("Update time");
    // Get the current time from the RTC
    DateTime now = rtc.now();

    // Getting each time field in individual variables
    // And adding a leading zero when needed;
    String yearStr = String(now.year(), DEC);
    String monthStr = (now.month() < 10 ? "0" : "") + String(now.month(), DEC);
    String dayStr = (now.day() < 10 ? "0" : "") + String(now.day(), DEC);
    String hourStr = (now.hour() < 10 ? "0" : "") + String(now.hour(), DEC);
    String minuteStr = (now.minute() < 10 ? "0" : "") + String(now.minute(), DEC);
    String secondStr = (now.second() < 10 ? "0" : "") + String(now.second(), DEC);
    String dayOfWeek = daysOfTheWeek[now.dayOfTheWeek()];

    // Complete time string
    // String formattedTime = dayOfWeek + ", " + yearStr + "-" + monthStr + "-" + dayStr + " " + hourStr + ":" + minuteStr + ":" + secondStr;
    String formattedDate = dayOfWeek + ", " + dayStr + "-" + monthStr + "-" + yearStr;
    String formattedTimeOnly = hourStr + ":" + minuteStr + ":" + secondStr;
    // Print the complete formatted time
    // Serial.println(formattedTime);

    // Vẽ một hình nền mờ (giả lập bằng màu xám nhạt)
    // tft.fillRect(24, 61, 136, 60, tft.color565(50, 50, 50));  // Xám mờ
    tft.setTextColor(TFT_WHITE, TFT_BLACK); // Chữ trắng, nền đen
    tft.setFreeFont(&FreeSerifBold9pt7b);
    tft.drawString(formattedDate, 29, 64);
    tft.setFreeFont(&FreeSerifBold18pt7b);
    tft.drawString(formattedTimeOnly, 30, 84);
    // Serial.println("End update time");
}

void update_main_dashboard_sensor()
{
    // Serial.println("Update sensor");
    float voltage, current, temperature;
    read_gateway_sensor_data(&voltage, &current, &temperature);

    String formattedVoltage = String(voltage) + "V";
    String formattedCurrent = String(current) + "mA";
    String formattedTemperature = String(temperature) + "°C";

    tft.setTextFont(1);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK); // Chữ trắng, nền trong suốt
    tft.drawString(formattedVoltage, 48, 149);
    tft.drawString(formattedCurrent, 48, 187);
    // tft.drawString("50%", 196, 139);

    tft.drawString(formattedTemperature, 265, 117);
    // Serial.println("End update sensor");

    // String formattedRSSI = "RSSI: " + String(packageRSSI);
    // String formattedSNR = "SNR" + String(packageSNR);

    // tft.setTextFont(1);
    // tft.setTextSize(1);
    // tft.setTextColor(TFT_WHITE, TFT_BLACK); // Chữ trắng, nền trong suốt
    // tft.drawString(formattedRSSI, 200, 149);
    // tft.drawString(formattedSNR, 200, 187);
}

void update_main_dashboard_communicate()
{
    tft.setTextFont(1);
    tft.setTextSize(1);
    tft.drawString(WiFi.SSID(), 132, 187);
    tft.drawString("Idle", 138, 149);
}

void update_control_dashboard(uint8_t order)
{
    if (RELAY_1[order - 1])
    {
        tft.fillCircle(47, 73, 20, RELAY_ON_COLOR);
        tft.drawCentreString("ON", 47, 63, 2);
    }
    else
    {
        tft.fillCircle(47, 73, 20, RELAY_OFF_COLOR);
        tft.drawCentreString("OFF", 47, 63, 2);
    }

    if (RELAY_2[order - 1])
    {
        tft.fillCircle(161, 73, 20, RELAY_ON_COLOR);
        tft.drawCentreString("ON", 161, 63, 2);
    }
    else
    {
        tft.fillCircle(161, 73, 20, RELAY_OFF_COLOR);
        tft.drawCentreString("OFF", 161, 63, 2);
    }

    if (RELAY_3[order - 1])
    {
        tft.fillCircle(272, 73, 20, RELAY_ON_COLOR);
        tft.drawCentreString("ON", 272, 63, 2);
    }
    else
    {
        tft.fillCircle(272, 73, 20, RELAY_OFF_COLOR);
        tft.drawCentreString("OFF", 272, 63, 2);
    }

    if (RELAY_4[order - 1])
    {
        tft.fillCircle(104, 163, 20, RELAY_ON_COLOR);
        tft.drawCentreString("ON", 104, 153, 2);
    }
    else
    {
        tft.fillCircle(104, 163, 20, RELAY_OFF_COLOR);
        tft.drawCentreString("OFF", 104, 153, 2);
    }

    if (RELAY_5[order - 1])
    {
        tft.fillCircle(215, 163, 20, RELAY_ON_COLOR);
        tft.drawCentreString("ON", 215, 153, 2);
    }
    else
    {
        tft.fillCircle(215, 163, 20, RELAY_OFF_COLOR);
        tft.drawCentreString("OFF", 215, 153, 2);
    }
}
#endif
