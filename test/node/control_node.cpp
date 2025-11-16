#define CONTROL_NODE

#include <LoRa.h>
#include <sensor_config.h>
#include <device_config.h>

// uint8_t mymac[6];
// Kết nối wifi để lấy MAC
// const char ssid[] = "P511A";
// const char pass[] = "passcua319b";
// const char ssid[] = "Phong209";
// const char pass[] = "pttkdtvt";
// const char ssid[] = "HNA";
// const char pass[] = "12345678";

#define PACKET_TIMEOUT 1000     // thời gian chờ giữa các packet

QueueHandle_t loraPackage;
unsigned long packetTime = 0;    // thời gian hiện tại của packet
bool finishDecodePkg = false;     // xử lý hết packet rồi mới vào loop

TaskHandle_t HandleLoRaProcessTask;
TaskHandle_t HandleLoRaReceiveTask;

void LoRaProcessTask(void *pvParameters)
{
    for (;;)
    {
        int numPackets = uxQueueMessagesWaiting(loraPackage);

        if (numPackets > 0)
        {
            // Nếu đã đủ thời gian không nhận thêm packet, thì bắt đầu xử lý batch
            if (millis() - packetTime > PACKET_TIMEOUT)
            {
                Serial.printf("🧮 Start decoding batch (%d packets)\n", numPackets);
                
                while (uxQueueMessagesWaiting(loraPackage) > 0)
                {
                    Packet pkt;
                    if (xQueueReceive(loraPackage, &pkt, 0) == pdTRUE)
                    {
                        decodeControlPacket(pkt.data, pkt.len, mac_ctrl_node[0]);
                    }
                }
                Serial.println("✅ Finished decoding batch");
                finishDecodePkg = true;
            }
        }
        checkAutoOffRelays();
        vTaskDelay(pdMS_TO_TICKS(200)); // Giảm tần suất kiểm tra
    }
}

void LoRaReceiveTask(void *pvParameters)
{
    for (;;)
    {
        int packetSize = LoRa.parsePacket();
        if (packetSize)
        {
            Packet pkt;
            int bytesRead = 0;
            while (LoRa.available() && bytesRead < packetSize)
            {
                pkt.data[bytesRead++] = LoRa.read();
            }
            pkt.len = bytesRead;

            Serial.print("📦 Received packet: ");
            for (int i = 0; i < pkt.len; i++)
            {
                Serial.printf("%02X ", pkt.data[i]);
            }
            Serial.println();

            packetTime = millis(); // cập nhật thời gian nhận mới nhất

            if (xQueueSend(loraPackage, &pkt, 0) != pdPASS)
            {
                Serial.println("⚠️ Queue full, packet dropped");
            }
            else
            {
                Serial.println("📥 Packet pushed to queue");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // để tránh chiếm CPU
    }
}


void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("Control Node");

    //   Serial.println("Connecting to WiFi...");
    //   WiFi.begin(ssid, pass);
    //   while (WiFi.status() != WL_CONNECTED)
    //   {
    //     delay(500);
    //     Serial.print(".");
    //   }
    //   Serial.println("\nWiFi connected!");

    //   getMacAddress(mymac);
    printMacAddress(mac_ctrl_node[0]);

    // in ra kiểm tra gửi thành công
    Serial.print("📤 Sent CONFIG_REQ Packet: ");
    pinMode(16, OUTPUT);
    digitalWrite(16, HIGH);
    loraInit();                               // Khởi tạo LoRa
    loraConfig(433E6, 14, 7, 125E3, 0xA5, 5); // Cấu hình LoRa: tần số 433MHz, công suất phát 17dBm, SF7, BW125kHz, sync word 0xA5, CR5 (4/5)
    
    loraPackage = xQueueCreate(10, sizeof(Packet));
    if (!loraPackage)
    {
        Serial.println("❌ Failed to create Queue");
        while (1)
            ;
    }

    delay(5000); // Đợi lâu tại bên kia setup lâu, sau code lại gateway RTOS + queue
    sendControlPacket(CONFIG_REQ, mac_ctrl_node[0]);
    initRelay();

    xTaskCreatePinnedToCore(
        LoRaProcessTask,        // Hàm xử lý
        "LoRaProcessTask",      // Tên Task
        4096,          // Stack size
        NULL,           // Tham số truyền vào
        3,              // Mức ưu tiên (1 = thấp)
        &HandleLoRaProcessTask, // Handle
        1               // Chạy trên core 1
    );

    xTaskCreatePinnedToCore(
        LoRaReceiveTask,
        "LoRaReceiveTask",
        4096,
        NULL,
        1,
        &HandleLoRaReceiveTask,
        1
    );
}

void loop()
{
    
}