#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "sensor_config.h"
#include "device_config.h"

// UUID tùy chỉnh (bạn có thể đổi UUID nếu muốn)
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic = nullptr;
BLEAdvertising *pAdvertising = nullptr;

extern float EC_target, pH_target, totalVolume_ml;
extern volatile bool BLE_finished;

// === Tách chuỗi dữ liệu BLE ===
void parseInput(String input)
{
    input.trim();
    int idx1 = input.indexOf(',');
    int idx2 = input.lastIndexOf(',');
    if (idx1 < 0 || idx2 < 0 || idx1 == idx2)
        return;

    EC_target = input.substring(0, idx1).toFloat();
    pH_target = input.substring(idx1 + 1, idx2).toFloat();
    totalVolume_ml = input.substring(idx2 + 1).toFloat();

    Serial.println("[📥] Nhận dữ liệu BLE:");
    Serial.printf("  EC_target: %.2f\n", EC_target);
    Serial.printf("  pH_target: %.2f\n", pH_target);
    Serial.printf("  V_total  : %.2f\n", totalVolume_ml);

    calculatePumpTime();
    // BLE_finished = true;
}

// === BLE characteristic callback ===
class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pChar) override
    {
        std::string rxValue = pChar->getValue();
        if (!rxValue.empty())
        {
            String input = String(rxValue.c_str());
            parseInput(input);
        }
    }
};

// === BLE server callback để tự quảng bá lại sau disconnect ===
class ServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer) override
    {
        Serial.println("[🔗] Thiết bị BLE đã kết nối.");
    }

    void onDisconnect(BLEServer *pServer) override
    {
        Serial.println("[⚠️] BLE ngắt kết nối.");
        pAdvertising->stop();
        BLE_finished = true;
        // delay(100);
        // if (pAdvertising != nullptr)
        // {
        //     pAdvertising->start();
        // }
    }
};

// === Gọi trong setup để khởi tạo BLE ===
void initBLE()
{
    Serial.println("[🔧] Khởi động BLE...");
    BLEDevice::init("EC_pH_Mixer_BLE");

    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE);
    pCharacteristic->setCallbacks(new MyCallbacks());

    pService->start();

    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->start();

    Serial.println("[📡] BLE đang quảng bá...");
}



