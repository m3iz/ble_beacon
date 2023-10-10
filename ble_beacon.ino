#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEScan.h>
#include <BLE2902.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

BLEServer* pServer;
BLECharacteristic* pCharacteristic;

// Ваши UUID для сервиса и характеристики
#define SERVICE_UUID        "0000180f-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "00002a19-0000-1000-8000-00805f9b34fb"

BLEScan* pBLEScan;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.print("BLE МАК адрес: ");
      Serial.println(advertisedDevice.getAddress().toString().c_str());
    }
};

uint8_t newMACAddress[] = {0x10, 0x00, 0x00, 0x00, 0x01, 0x0a};

String knownMAC[numBeacons] = {
  "10:00:00:00:02:0c",                           
  "10:00:00:00:03:0c",                            
  "10:00:00:00:04:0c",
  "10:00:00:00:05:0c",                           
  "10:00:00:00:06:0c",                            
  "10:00:00:00:07:0c",  
  "10:00:00:00:08:0c",                           
  "10:00:00:00:09:0c",                            
  "10:00:00:00:10:0c",                              
};  

void scanTask(void *pvParameters) {
  for (;;) {
    BLEDevice::init("BLE_Scanner");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    BLEScanResults foundDevices = pBLEScan->start(1); // Сканирование на 5 секунд
    int count = foundDevices.getCount();    for (int j = 0; j < count; j++) {
    BLEAdvertisedDevice d = foundDevices.getDevice(j);
    String dMAC = d.getAddress().toString().c_str();      for (byte i = 0; i < numBeacons; i++) {
      if (dMAC == knownMAC[i]) {
        Serial.print("Найден знаковый MAC: ");
        Serial.println(dMAC);
        Serial.print("RSSI: ");
        Serial.println(d.getRSSI());
        if (d.getRSSI() > minRSSI) {
          BLINK_red();
          numNotFoundMAC[i] = 0;
        }          
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Подождите 1 секунду перед повторным сканированием
  }
}

void setup() {
  Serial.begin(115200);

  // Инициализация BLE сервера
  BLEDevice::init("BLE_Server");
  pServer = BLEDevice::createServer();
  esp_base_mac_addr_set(newMACAddress); //установка статического mac-адреса
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristic->setValue("Hello, Client!");
  pService->start();

  // Размещение сервера на определенной GATT службе и характеристике
  pServer->getAdvertising()->addServiceUUID(pService->getUUID());
  pServer->getAdvertising()->start();

  // Запуск задачи для сканирования
  xTaskCreate(scanTask, "ScanTask", 4096, NULL, 1, NULL);
}

void loop() {
  // Дополнительная логика BLE сервера может быть добавлена здесь
}



