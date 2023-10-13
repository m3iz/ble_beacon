#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEScan.h>
#include <BLE2902.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "blink.h"

//tasks сделать уникальный мак адресс fix + serial
#define MODE 2

#define SNUM 15

BLEServer* pServer;
BLECharacteristic* pCharacteristic;

int sval[SNUM];
int mval=0;
int it=0;

bool inZone = false;
int slevel = 1;
bool deviceFound = false;
//UUID для сервиса и характеристики
#define SERVICE_UUID        "0000180f-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "00002a19-0000-1000-8000-00805f9b34fb"

const int numBeacons = 10;

int counter = 0;
int decounter = 0;

const int minRSSI = -85; //-85

BLEScan* pBLEScan;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks { 
    void onResult(BLEAdvertisedDevice advertisedDevice) {
   // if(!inZone){  
    BLEAddress deviceAddress = advertisedDevice.getAddress();

    // Извлекаем строку MAC-адреса
    String macAddress = deviceAddress.toString().c_str();

    // Извлекаем только первые 8 символов (первые 3 октета)
    String firstThreeOctets = macAddress.substring(0, 8);

    // Проверяем, сравниваем с "10:00:00"
    if (firstThreeOctets.equals("10:00:00")) {
      Serial.println("Device with MAC address starting with 10:00:00 found!");
      pBLEScan-> stop();
    }
    }

      //Serial.print("BLE МАК адрес: ");
      //Serial.println(advertisedDevice.getAddress().toString().c_str());
     
   // }
};

void blinkTask(void *pvParameters) {
  while(1){
    BLINK_red();
  }
}
void scanTask(void *pvParameters) {
  for (;;) {
    Serial.print("Scanning...\n");
    deviceFound = false;
    BLEDevice::init("BLE_Scanner");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    BLEScanResults foundDevices = pBLEScan->start(1);  
    int count = foundDevices.getCount();    
    for (int j = 0; j < count; j++) 
    {
      BLEAdvertisedDevice d = foundDevices.getDevice(j);
      String dMAC = d.getAddress().toString().c_str();      
      String firstThreeOctets = dMAC.substring(0, 8);
      // Проверяем, сравниваем с "10:00:00"
      if (firstThreeOctets.equals("10:00:00")) {          
          Serial.print("Найден MAC: ");
          deviceFound = true;
          Serial.println(dMAC);
          Serial.print("RSSI: ");
          Serial.println(d.getRSSI());
          sval[it]=abs(d.getRSSI());
          it++;
          if(it==SNUM)it=0;
          int temp=0;
          for(int i=0;i<SNUM;i++){
            temp+=sval[i];
            Serial.print("i: "); Serial.print(i); Serial.print(" ");
            Serial.println(sval[i]);
          }
          mval=temp/SNUM;
          
          if ((d.getRSSI() > minRSSI) && (counter >= SNUM))
          {
            inZone = true;
            decounter = 0;
            slevel = abs(d.getRSSI());
            
          }
          else if(d.getRSSI() < minRSSI){
            decounter++;
            if(decounter>=SNUM-10){
               inZone = false;
               counter = 0;
            }
          }
                
          break;
        }
    }
  
  if(!deviceFound) {
    counter = 0;
    if(inZone)decounter++;
    if(decounter>=SNUM){
      inZone = false;
      
    }
  }
  else {
    counter++;
  }
  pBLEScan -> clearResults();
  
  //vTaskDelay(0); // Задержка перед сканированием
  }
}

void setup() {
  Serial.begin(115200);
  
  
  BLINK_init();
  helloBlink();
  // Инициализация BLE сервера
  
  uint64_t chipId = ESP.getEfuseMac();

  // Преобразовать последние 3 байта серийного номера в массив uint8_t
  uint8_t last3Bytes[3];
  last3Bytes[0] = (chipId >> 16) & 0xFF;
  last3Bytes[1] = (chipId >> 8) & 0xFF;
  last3Bytes[2] = chipId & 0xFF;

  // Создать MAC-адрес с первыми тремя октетами "10:00:00" и последними тремя октетами из last3Bytes
  uint8_t macAddress[] = {0x10, 0x00, 0x00, last3Bytes[0], last3Bytes[1], last3Bytes[2]};
  //uint8_t newMACAddress[] = {0x10, 0x00, 0x00, 0x00, 0x01, 0x0a};
  esp_base_mac_addr_set(macAddress);
  BLEDevice::init("Ble_device");
 // BLEDevice::setPower(ESP_PWR_LVL_P7); //ESP_PWR_LVL_P7
  pServer = BLEDevice::createServer();
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

  // Запуск задачи для сканирования и индикации
  xTaskCreate(scanTask, "ScanTask", 4096, NULL, 1, NULL);
  xTaskCreate(blinkTask, "BLINK_red", 4096, NULL, 2, NULL);
}

void loop() {
  // Дополнительная логика BLE сервера может быть добавлена здесь
}