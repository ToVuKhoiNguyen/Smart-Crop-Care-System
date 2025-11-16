#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// UUID tùy chỉnh (bạn có thể đổi UUID nếu muốn)
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Biến lưu giá trị target
int ec_target = 0;
float ph_target = 0.0;

// BLE Object
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

// BLE Server Callback
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("[BLE] Thiết bị đã kết nối.");
  }

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    Serial.println("[BLE] Thiết bị đã ngắt kết nối.");

    // Bắt đầu quảng bá lại BLE
    BLEDevice::startAdvertising();
    Serial.println("[BLE] Đang quảng bá lại...");
  }
};

// Hàm xử lý tách chuỗi
void parseInput(String input)
{
  input.trim(); // Xoá khoảng trắng thừa

  int spaceIndex = input.indexOf(' ');
  if (spaceIndex > 0)
  {
    String ecPart = input.substring(0, spaceIndex);
    String phPart = input.substring(spaceIndex + 1);

    ec_target = ecPart.toInt();
    ph_target = phPart.toFloat();

    Serial.print("[BLE] Đã phân tách: EC = ");
    Serial.print(ec_target);
    Serial.print(", pH = ");
    Serial.println(ph_target);
  }
  else
  {
    Serial.println("[BLE] Lỗi: Định dạng không hợp lệ (cần có dấu cách giữa EC và pH).");
  }
}

// BLE Characteristic Callback
class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    String rxValue = pCharacteristic->getValue(); // ⚡ đã sửa đúng kiểu String

    if (rxValue.length() > 0)
    {
      Serial.print("[BLE] Nhận dữ liệu: ");

      // Duyệt từng ký tự thủ công
      String inputData = "";
      for (size_t i = 0; i < rxValue.length(); i++)
      {
        inputData += rxValue[i];
        Serial.print(rxValue[i]);
      }
      Serial.println();

      // Gọi hàm xử lý dữ liệu
      parseInput(inputData);
    }
  }
};

void setup()
{
  Serial.begin(115200);
  Serial.println("[BLE] Khởi động ESP32 BLE Server...");

  BLEDevice::init("ESP32_BLE_Server"); // Tên thiết bị BLE

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("[BLE] ESP32 BLE Server đang quảng bá...");
}

void loop()
{
  // In giá trị target mỗi 5 giây để debug
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 5000)
  {
    lastPrint = millis();
    Serial.print("[BLE] Giá trị hiện tại: EC = ");
    Serial.print(ec_target);
    Serial.print(", pH = ");
    Serial.println(ph_target);
  }
}
