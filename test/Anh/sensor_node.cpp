#include <LoRa.h>
#include <Wire.h>
#include "sensor_config.h"  // Thư viện cho INA

#define SHT20_ADDRESS 0x40
#define SOIL_MOISTURE_PIN 13  // Chân analog đọc cảm biến độ ẩm đất

#define ss 5    // Slave Select pin
#define rst 4   // Reset pin
#define dio0 2  // DIO0 pin

void setup() {
  Wire.begin(); // SDA = GPIO21, SCL = GPIO22 (mặc định trên ESP32)
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Sensor Node - SHT20, Soil Moisture & INA219 Test");

  init_current_sensor(); // Khởi tạo INA219

  LoRa.setPins(ss, rst, dio0);
  while (!LoRa.begin(433E6)) { // 433 MHz
    Serial.println("LoRa initialization failed...");
    delay(500);
  }
  LoRa.setSyncWord(0xA5);
  Serial.println("LoRa Initializing OK!");
}

float readTemperature() {
  Wire.beginTransmission(SHT20_ADDRESS);
  Wire.write(0xF3); // Command to measure temperature
  if (Wire.endTransmission() != 0) return -999; // Error

  delay(100); // Wait for measurement
  Wire.requestFrom(SHT20_ADDRESS, 2);
  if (Wire.available() != 2) return -999; // Error

  uint16_t rawData = (Wire.read() << 8) | Wire.read();
  return -46.85 + 175.72 * (rawData / 65536.0);
}

float readHumidity() {
  Wire.beginTransmission(SHT20_ADDRESS);
  Wire.write(0xF5); // Command to measure humidity
  if (Wire.endTransmission() != 0) return -999; // Error

  delay(100); // Wait for measurement
  Wire.requestFrom(SHT20_ADDRESS, 2);
  if (Wire.available() != 2) return -999; // Error

  uint16_t rawData = (Wire.read() << 8) | Wire.read();
  return -6.0 + 125.0 * (rawData / 65536.0);
}

int readSoilMoisture() {
  return analogRead(SOIL_MOISTURE_PIN); // Đọc giá trị analog
}

void sendLoRaData(float temp, float hum, int soil, float voltage, float current) {
  uint8_t buffer[14];

  buffer[0] = 0x4E; // N
  buffer[1] = 0x57; // W
  buffer[2] = 0x01; // Sensor Node
  buffer[3] = 0x01; // Lệnh gửi dữ liệu

  uint16_t tempInt = (uint16_t)(temp * 10);
  uint16_t humInt = (uint16_t)(hum * 10);
  uint16_t soilInt = (uint16_t)soil;
  uint16_t voltInt = (uint16_t)(voltage * 100);
  uint16_t currInt = (uint16_t)(current * 100);

  buffer[4] = (tempInt >> 8) & 0xFF;
  buffer[5] = tempInt & 0xFF;
  buffer[6] = (humInt >> 8) & 0xFF;
  buffer[7] = humInt & 0xFF;
  buffer[8] = (soilInt >> 8) & 0xFF;
  buffer[9] = soilInt & 0xFF;
  buffer[10] = (voltInt >> 8) & 0xFF;
  buffer[11] = voltInt & 0xFF;
  buffer[12] = (currInt >> 8) & 0xFF;
  buffer[13] = currInt & 0xFF;

  LoRa.beginPacket();
  LoRa.write(buffer, sizeof(buffer));
  LoRa.endPacket();

  Serial.println("Data sent via LoRa");
}

void loop() {
  float temperature = readTemperature();
  float humidity = readHumidity();
  int soilMoisture = readSoilMoisture();
  float voltage = get_sourse_voltage_V();
  float current = get_current_mA();

  if (temperature != -999 && humidity != -999) {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
    Serial.print("Soil Moisture: ");
    Serial.print(soilMoisture);
    Serial.println(" (analog value)");
    Serial.print("Voltage: ");
    Serial.print(voltage);
    Serial.println(" V");
    Serial.print("Current: ");
    Serial.print(current);
    Serial.println(" mA");

    // Gửi dữ liệu qua LoRa
    sendLoRaData(temperature, humidity, soilMoisture, voltage, current);
  } else {
    Serial.println("Failed to read from sensor");
  }

  delay(5000); // Gửi mỗi 5 giây
}
