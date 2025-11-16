/* Define MQTT host */
#define DEFAULT_MQTT_HOST "mqtt1.eoh.io"

// You should get Auth Token in the ERa App or ERa Dashboard
#define ERA_AUTH_TOKEN "0ae424b1-ebd6-497c-9680-d19428d57778"

#include <LoRa.h>
#include <ERa.hpp>
#include <Wire.h>
#include <RTClib.h>
#include <sensor_config.h>
#include <device_config.h>

// const char ssid[] = "P511A";
// const char pass[] = "passcua319b";
// const char ssid[] = "Phong209";
// const char pass[] = "pttkdtvt";
const char ssid[] = "512B";
const char pass[] = "1123581321";

WiFiClient mbTcpClient;

volatile bool hasIncomingPacket = false; // Cờ báo có gói tin đến
uint8_t incomingBuffer[128];             // Buffer để lưu gói tin đến
int incomingLength = 0;                  // Độ dài gói tin đến

/* This function will run every time ERa is connected */
ERA_CONNECTED()
{
    ERA_LOG(ERA_PSTR("ERa"), ERA_PSTR("ERa connected!"));
}

/* This function will run every time ERa is disconnected */
ERA_DISCONNECTED()
{
    ERA_LOG(ERA_PSTR("ERa"), ERA_PSTR("ERa disconnected!"));
}

// hàm xử lý lệnh nhận
void onReceive(int packetSize)
{
    memset(incomingBuffer, 0, sizeof(incomingBuffer)); // Đặt lại buffer về 0 để tránh lỗi khi nhận gói mới
    // Serial.printf("📥 LoRa onReceive triggered, packetSize = %d\n", packetSize);

    if (packetSize == 0 || hasIncomingPacket)
        return; // bỏ qua nếu đang xử lý gói trước

    int i = 0;
    while (LoRa.available() && i < sizeof(incomingBuffer))
    {
        incomingBuffer[i++] = LoRa.read();
    }
    incomingLength = i;
    hasIncomingPacket = true;
}

// Nhận lệnh từ web điều khiển relay
ERA_WRITE(V3)
{
    int value = param.getInt();
    Serial.print("Received command for Relay 1: ");
    Serial.println(value);
    sendControlCommand(mac_ctrl_node[0], 1, uint8_t(value), 0);
    ERa.virtualWrite(V3, value);
}

ERA_WRITE(V4)
{
    int value = param.getInt();
    Serial.print("Received command for Relay 3: ");
    Serial.println(value);
    sendControlCommand(mac_ctrl_node[0], 3, uint8_t(value), 0);
    ERa.virtualWrite(V4, value);
}

ERA_WRITE(V9)
{
    int value = param.getInt();
    Serial.print("Received command for Relay 5: ");
    Serial.println(value);
    sendControlCommand(mac_ctrl_node[0], 5, uint8_t(value), 0);
    ERa.virtualWrite(V4, value);
}

TaskHandle_t Era;
void Task1_Era(void *parameter)
{
    //   Serial.println(xPortGetCoreID());
    /* Setup Client for Modbus TCP/IP */
    ERa.setModbusClient(mbTcpClient);

    /* Set scan WiFi. If activated, the board will scan
        and connect to the best quality WiFi. */
    ERa.setScanWiFi(true);
    Serial.println("Initial setup complete.");

    /* Initializing the ERa library. */
    ERa.begin(ssid, pass);
    for (;;)
    {
        ERa.run();
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("Gateway Node");

    loraInit();                               // Khởi tạo LoRa
    loraConfig(433E6, 14, 7, 125E3, 0xA5, 5); // Cấu hình LoRa: tần số 433MHz, công suất phát 17dBm, SF7, BW125kHz, sync word 0xA5, CR5 (4/5)

    // LoRa.onReceive(onReceive); // gán callback để xử lý khi nhận dữ liệu
    LoRa.idle();               // Bắt đầu nhận dữ liệu

    delay(5000); // Đợi Sensor kịp khởi động và ổn định LoRa
    xTaskCreatePinnedToCore(Task1_Era, "Era", 10000, NULL, 1, &Era, 1);
    // delay(500);

    // sendNodePacket(CONFIG_REQ, mac_sens_node[0]);
    sendNodePacket(CONFIG_REQ, mac_sens_node[1]);
}

void loop()
{
    ERa.run();

    int packetSize = LoRa.parsePacket();
    if (packetSize)
    {
        Serial.print("Packet size: ");
        Serial.println(packetSize);

        uint8_t buffer[128]; // Tăng kích thước buffer để nhận thêm dữ liệu
        int bytesRead = 0;
        while (LoRa.available() && bytesRead < packetSize)
        {
            buffer[bytesRead++] = LoRa.read();
        }

        Serial.print("Received bytes: ");
        for (int i = 0; i < bytesRead; i++)
        {
            Serial.print(buffer[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        decodeIncomingPacket(buffer, bytesRead); // Giải mã gói tin đến
    }
}