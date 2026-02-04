#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEScan.h>
#include <BLE2902.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <map>
#include <array>
#include <vector>
#include "blink.h"
#include "esp_mac.h"

// ====== Конфигурация ======
HardwareSerial SerialSTM(2);

#define MODE 2
#define RLEVEL 3
#define SNUM 10      // размер буфера RSSI для каждого MAC
#define REPCOR 15    // коррекция RSSI для специального MAC
#define SERVICE_UUID        "0000180f-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "00002a19-0000-1000-8000-00805f9b34fb"
#define RX_BUF_SIZE 256
// Пороговые значения (в абсолютных значениях: abs(RSSI))
const int minRSSI = 75;   // соответствует -75 dBm -> использовать abs()
const int minrRSSI = 60;  // более строгий порог

// ====== Глобальные переменные состояния ======
uint64_t chipId = 0;
uint8_t last3Bytes[3];

BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
BLEScan* pBLEScan = nullptr;

std::map<String, std::vector<int>> rssiData;
std::map<String, std::array<int,3>> lastData; // [0]=avgRSSI, [1]=inRowFlag, [2]=inZoneFlag

// локальные флаги/счётчики
int mval = 100;       // минимальное среднее на текущем проходе
int rcounter = 0;
bool inZone = false;
bool deviceFound = false;
bool inrow = false;
int radio_state = 0;
int counter = 0;
int decounter = 0;
int dcounter = 0;
int ledcounter = 0;
int leddecounter = 0;

// Переменные для индикации (BLINK)
extern bool led; // предполагается, что BLINK управляет этой переменной/функцией

// ====== Вспомогательные функции ======
int midval(int *arr, int len){
  long sum = 0;
  for(int i=0;i<len;i++){
    sum += arr[i];
  }
  return (int)(sum / len);
}

// Возвращает первые 8 символов MAC в формате "AA:BB:CC"
static String firstThreeOctets(const String &mac) {
  if (mac.length() >= 8) return mac.substring(0,8);
  return mac;
}

// ====== Задача мигания (оставлена как было) ======
void blinkTask(void *pvParameters) {
  (void)pvParameters;
  while(1){
    BLINK_red();
    vTaskDelay(pdMS_TO_TICKS(100)); // небольшой sleep чтобы не жрать CPU
  }
}

// ====== Задача сканирования (основная логика) ======
void scanTask(void *pvParameters) {
  (void)pvParameters;
  const uint32_t scanSeconds = 2; // сканируем по 2 секунды — стабильный вариант
  while (true) {
    inrow = false;
    deviceFound = false;

    // Запуск синхронного сканирования (блокирует на scanSeconds)
    BLEScanResults* foundDevices = pBLEScan->start(scanSeconds, false); // duration в секундах

    int foundCount = foundDevices-> getCount();

    // временное значение для минимального avgRSSI для этого прохода
    int tval = 100;

    // Проходим по всем найденным устройствам
    for (int j = 0; j < foundCount; j++) {
      BLEAdvertisedDevice d = foundDevices->getDevice(j);
      String dMAC = d.getAddress().toString().c_str();
      String first3 = firstThreeOctets(dMAC);

      // фильтрация — ты хотел только те, у кого первые октеты "10:00:00"
      if (first3.equals("10:00:00")) {
        deviceFound = true;
        int rawRssi = d.getRSSI();
        int absRssi = abs(rawRssi);

        // специальная коррекция для конкретного MAC (как в оригинале)
        if (dMAC.equals("10:00:00:00:00:00")) {
          absRssi = max(0, absRssi - REPCOR);
        }

        // инициализация буфера если нужно
        auto it = rssiData.find(dMAC);
        if (it == rssiData.end()) {
          rssiData[dMAC] = std::vector<int>();
        }
        // добавляем новое значение
        rssiData[dMAC].push_back(absRssi);

        // ограничиваем длину буфера
        if (rssiData[dMAC].size() > SNUM) {
          rssiData[dMAC].erase(rssiData[dMAC].begin());
        }

        // Вычисляем среднее и счётчики для данного MAC
        int sum = 0;
        int cnt_leq = 0;
        int cnt_gt = 0;
        for (int v : rssiData[dMAC]) {
          sum += v;
          if (v <= minRSSI) cnt_leq++;
          else cnt_gt++;
        }
        int avg = (rssiData[dMAC].size() > 0) ? (sum / (int)rssiData[dMAC].size()) : 100;

        // Гарантируем, что lastData имеет запись
        if (lastData.find(dMAC) == lastData.end()) {
          lastData[dMAC] = {100, 0, 0};
        }
        lastData[dMAC][0] = avg;

        // inRow: проверяем последние 3 элементов в буфере (если есть)
        bool inRowFlag = false;
        if (rssiData[dMAC].size() >= 3) {
          size_t s = rssiData[dMAC].size();
          if (rssiData[dMAC][s-1] <= minrRSSI &&
              rssiData[dMAC][s-2] <= minrRSSI &&
              rssiData[dMAC][s-3] <= minrRSSI) {
            inRowFlag = true;
          }
        }
        lastData[dMAC][1] = inRowFlag ? 1 : 0;

        // inZone: если весь буфер <= minRSSI
        if (cnt_leq >= (int)rssiData[dMAC].size()) {
          lastData[dMAC][2] = 1;
        } else if (cnt_gt >= (int)rssiData[dMAC].size()) {
          lastData[dMAC][2] = 0;
        }
        // После обработки одного релевантного устройства выходим из перебора,
        // как в оригинале у тебя стоял break — чтобы учитывать только один (первый) найденный relevant device
        break;
      }
    } // for foundDevices

    // Проходим по lastData чтобы вычислить глобальные флаги
    for (const auto &pair : lastData) {
      const auto &arr = pair.second;
      if (arr[1] == 1) inrow = true;
      if (arr[2] == 1) inZone = true;
      if (arr[0] < tval) tval = arr[0];
    }

    // логика сравнения минимального значения
    if (tval == mval) {
      rcounter++;
    } else rcounter = 0;

    if (rcounter > 35) {
      // ресет данных если ничего не меняется длительное время
      lastData.clear();
      rssiData.clear();
      rcounter = 0;
    }
    mval = tval;

    // логика индикации светодиода (как было)
    if (mval <= minrRSSI) {
      ledcounter++;
      leddecounter = 0;
    } else {
      leddecounter++;
      ledcounter = 0;
    }
    if ((ledcounter >= RLEVEL) && (inrow)) led = true;
    else if (leddecounter >= RLEVEL + 3) led = false;

    // Если в этом цикле не нашли ни одного релевантного девайса
    if (!deviceFound) {
      if (inZone) dcounter++;
      if (dcounter >= SNUM - 10) {
        // обнуляем зону
        inZone = false;
        dcounter = 0;
        mval = 100;
        rssiData.clear();
        lastData.clear();
      }
    }

    // очистка результатов (ресурсо-освобождение)
    pBLEScan->clearResults();

    // небольшая пауза чтобы не перегружать цикл (и дать другим задачам поработать)
    vTaskDelay(pdMS_TO_TICKS(10));
  } // while
}
// ====== НОВАЯ ЗАДАЧА — чтение UART от STM32 ======
void uartReadTask(void *pvParameters)
{
    static char rxbuf[RX_BUF_SIZE];
    static uint16_t len = 0;

    while (true)
    {
        while (SerialSTM.available())
        {
            char c = SerialSTM.read();

            if (len < RX_BUF_SIZE - 1)
                rxbuf[len++] = c;

            rxbuf[len] = 0;

            if (strstr(rxbuf, "RADIO NEAR"))
            {
                radio_state = 1;
                Serial.println("radio_state = 1;");
                len = 0;
                memset(rxbuf, 0, sizeof(rxbuf));
            }
            else if (strstr(rxbuf, "RADIO LOST") ||
                     strstr(rxbuf, "RADIO FAR"))
            {
                radio_state = 0;
                Serial.println("RADIO = LOST/FAR");
                len = 0;
                memset(rxbuf, 0, sizeof(rxbuf));
            }

            // защита от мусора
            if (len >= RX_BUF_SIZE - 2)
            {
                len = 0;
                memset(rxbuf, 0, sizeof(rxbuf));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
// ====== setup() — инициализация BLE и задач ======
void setup() {
  //Serial.begin(115200);
    SerialSTM.begin(115200, SERIAL_8N1, 18, 17);
  Serial.println("UART2 (SerialSTM) запущен: RX=GPIO18, TX=GPIO17");
  BLINK_init();
  helloBlink();

  // Настроим MAC как у тебя было
  chipId = ESP.getEfuseMac();
  last3Bytes[0] = (chipId >> 24) & 0xFF;
  last3Bytes[1] = (chipId >> 32) & 0xFF;
  last3Bytes[2] = (chipId >> 40) & 0xFF;
  uint8_t macAddress[] = {0x10, 0x00, 0x00, last3Bytes[0], last3Bytes[1], last3Bytes[2]};
  esp_base_mac_addr_set(macAddress);

  // Инициализация BLE — только один раз
  BLEDevice::init("Ble_device2");

  // Создаём сервер и характеристику (как было)
  pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristic->setValue("Hello, Client!");
  pService->start();
  pServer->getAdvertising()->addServiceUUID(pService->getUUID());
  pServer->getAdvertising()->start();

  // Настройка сканера — делаем это один разну я
  pBLEScan = BLEDevice::getScan();
  // Настроим параметры: interval/window — помогают стабилизировать сканирование
  // Значения в миллисекундах; метод принимает числа в тиках/условностях библиотеки — стандартные примеры используют такие значения.
  pBLEScan->setInterval(1349); // пример из примеров — увеличиваем промежуток
  pBLEScan->setWindow(449);    // окно сканирования
  pBLEScan->setActiveScan(true); // можно оставить activeScan=true, чтобы получать расширенные данные (при проблемах — поставить false)

  // Создадим задачи
  xTaskCreate(scanTask, "ScanTask", 8192, NULL, 1, NULL);
  xTaskCreate(blinkTask, "BLINK_red", 2048, NULL, 2, NULL);
  xTaskCreate(uartReadTask, "UartRead", 3072, NULL, 1, NULL);
}

void loop() {
  Serial.println("v1.0.0");
  // Не используем loop для BLE — вся логика в задачах
  vTaskDelay(pdMS_TO_TICKS(10000));
}
