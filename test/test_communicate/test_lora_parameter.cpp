#include <SPI.h>
#include <LoRa.h>

// Chân kết nối (có thể thay đổi nếu bạn dùng chân khác)
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

// Thống kê gói tin
unsigned long totalPackets = 0;
unsigned long successPackets = 0;

void setup() {
  Serial.begin(115200);

  // Setup chân
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(10);

  // Bắt đầu LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed. Check connections.");
    while (1);
  }

  // Cấu hình thêm nếu cần
  LoRa.setSpreadingFactor(7);   // hoặc 12 cho truyền xa
  LoRa.setSignalBandwidth(125E3); // hoặc 62.5E3
  LoRa.setCodingRate4(5);       // hoặc 8
  LoRa.setTxPower(14);          // hoặc 17
  LoRa.setSyncWord(0xA5);

  Serial.println("LoRa Receiver ready.");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    totalPackets++;

    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }

    // Lấy RSSI, SNR
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    successPackets++;
    float successRate = (float)successPackets / totalPackets * 100.0;

    Serial.println("------ PACKET RECEIVED ------");
    Serial.println("Data: " + incoming);
    Serial.print("RSSI: "); Serial.print(rssi); Serial.println(" dBm");
    Serial.print("SNR: "); Serial.print(snr); Serial.println(" dB");
    Serial.print("Success Rate: "); Serial.print(successRate); Serial.println(" %");
    Serial.println("-----------------------------");
  }

  // Optional: timeout nếu không nhận được sau thời gian
}