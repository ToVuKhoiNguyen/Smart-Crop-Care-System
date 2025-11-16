#define DEFAULT_MQTT_HOST "mqtt1.eoh.io"
#define ERA_AUTH_TOKEN "0ae424b1-ebd6-497c-9680-d19428d57778" // Thay bằng token của bạn

#include <LoRa.h>
#include <ERa.hpp>

const char ssid[] = "P511A"; // Thay bằng SSID WiFi của bạn
const char pass[] = "passcua319b"; // Thay bằng mật khẩu WiFi

#define ss 5
#define rst 4
#define dio0 2

WiFiClient mbTcpClient;

ERA_CONNECTED() {
  ERA_LOG(ERA_PSTR("ERa"), ERA_PSTR("ERa connected!"));
}

ERA_DISCONNECTED() {
  ERA_LOG(ERA_PSTR("ERa"), ERA_PSTR("ERa disconnected!"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Gateway Node");

  // Khởi tạo LoRa
  LoRa.setPins(ss, rst, dio0);
  while (!LoRa.begin(433E6)) {
    Serial.print("Waiting for LoRa initialization");
    Serial.print(".");
    delay(1000);
  }
  LoRa.setSyncWord(0xA5);
  Serial.println("LoRa Initializing OK!");

  // Kết nối ERA IoT
  ERa.setModbusClient(mbTcpClient);
  ERa.setScanWiFi(true);
  ERa.begin(ssid, pass);
}

void loop() {
  ERa.run();

  // Nhận dữ liệu từ LoRa
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Packet size: ");
    Serial.println(packetSize);

    uint8_t buffer[14];  // Tăng kích thước buffer
    int bytesRead = 0;
    while (LoRa.available() && bytesRead < packetSize) {
      buffer[bytesRead++] = LoRa.read();
    }

    Serial.print("Received bytes: ");
    for (int i = 0; i < packetSize; i++) {
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Kiểm tra header và lệnh
    if (buffer[0] == 0x4E && buffer[1] == 0x57 && buffer[3] == 0x01) {
      // Giải mã dữ liệu
      uint16_t tempInt = (buffer[4] << 8) | buffer[5];
      uint16_t humInt = (buffer[6] << 8) | buffer[7];
      uint16_t soilMoistureInt = (buffer[8] << 8) | buffer[9];
      float temperature = tempInt / 10.0;
      float humidity = humInt / 10.0;
      float soilMoisture = soilMoistureInt / 10.0;
      uint16_t voltageInt = (buffer[10] << 8) | buffer[11];
      uint16_t currentInt = (buffer[12] << 8) | buffer[13];
      float voltage = voltageInt / 100.0;
      float current = currentInt / 100.0;

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
      Serial.println(" mA");

      // Gửi lên ERA IoT
      ERa.virtualWrite(V0, temperature); // Gửi nhiệt độ lên V0
      ERa.virtualWrite(V1, humidity);    // Gửi độ ẩm lên V1
      ERa.virtualWrite(V2, soilMoisture); // Gửi độ ẩm đất lên V2
      ERa.virtualWrite(V5, voltage); // Gửi điện áp lên V5
      ERa.virtualWrite(V6, current); // Gửi dòng điện lên V6
    }
  }
}
