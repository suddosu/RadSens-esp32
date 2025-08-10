// Объединённый скетч: RadSens + IP5306 -> BLE (совместимо с Android-приложением "Fast/AtomFast")
// ESP32 core 2.0.17!!! На 3.3.0 не взлетит

#include <Arduino.h>
#include "CG_RadSens.h"
#include <Wire.h>
#include <GyverOLED.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ====================== UUID'ы BLE ======================
#define SERVICE_UUID "63462a4a-c28c-4ffd-87a4-2d23a1c72581"
BLECharacteristic DoseCharacteristics("70bc767e-7a1a-4304-81ed-14b9af54f7bd", BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic ThresholdCharacteristics_1("3f71e820-1d98-46d4-8ed6-324c8428868c", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE );
BLECharacteristic ThresholdCharacteristics_2("2e95d467-4db7-4d7f-9d82-4cd5c102fa05", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE );
BLECharacteristic ThresholdCharacteristics_3("f8de242f-8d84-4c12-9a2f-9c64a31ca7ca", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE );
BLECharacteristic SettingsCharacteristics("ea50cfcd-ac4a-4a48-bf0e-879e548ae157", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE );

bool deviceConnected = false;

// ====================== IP5306 (I2C) регистры и вспомогательные ======================
#define IP5306_REG_SYS_0    0x00
#define IP5306_REG_SYS_1    0x01
#define IP5306_REG_SYS_2    0x02
#define IP5306_REG_CHG_0    0x20
#define IP5306_REG_CHG_1    0x21
#define IP5306_REG_CHG_2    0x22
#define IP5306_REG_CHG_3    0x23
#define IP5306_REG_CHG_4    0x24
#define IP5306_REG_READ_0   0x70
#define IP5306_REG_READ_1   0x71
#define IP5306_REG_READ_2   0x72
#define IP5306_REG_READ_3   0x77
#define IP5306_REG_READ_4   0x78

#define IP5306_GetPowerSource()                 ip5306_get_bits(IP5306_REG_READ_0, 3, 1)//0:BAT, 1:VIN
#define IP5306_GetBatteryFull()                 ip5306_get_bits(IP5306_REG_READ_1, 3, 1)//0:CHG/DIS, 1:FULL
#define IP5306_GetLevelLeds()                ((~ip5306_get_bits(IP5306_REG_READ_4, 4, 4)) & 0x0F)//LED[0-4] State (inverted)
#define IP5306_LEDS2PCT(byte)  \
  ((byte & 0x01 ? 25 : 0) + \
  (byte & 0x02 ? 25 : 0) + \
  (byte & 0x04 ? 25 : 0) + \
  (byte & 0x08 ? 25 : 0))

int ip5306_get_reg(uint8_t reg){
    Wire.beginTransmission(0x75);
    Wire.write(reg);
    if(Wire.endTransmission(false) == 0 && Wire.requestFrom(0x75, 1)){
        return Wire.read();
    }
    return -1;
}

int ip5306_set_reg(uint8_t reg, uint8_t value){
    Wire.beginTransmission(0x75);
    Wire.write(reg);
    Wire.write(value);
    if(Wire.endTransmission(true) == 0){
        return 0;
    }
    return -1;
}

uint8_t ip5306_get_bits(uint8_t reg, uint8_t index, uint8_t bits){
    int value = ip5306_get_reg(reg);
    if(value < 0){
        //Serial.printf("ip5306_get_bits fail: 0x%02x\n", reg);
        return 0;
    }
    return (value >> index) & ((1 << bits)-1);
}

// ====================== RadSens ======================
CG_RadSens radSens(RS_DEFAULT_I2C_ADDRESS);

// ====================== OLED ======================
GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;
char *connstat;

// переменные данных
float dynval = 0;   // динамическая интенсивность
float statval = 0;  // статическая интенсивность / доза
uint32_t impval = 0; // общее количество импульсов (счетчик)
uint32_t pulsesPrev = 0;
uint8_t verval = 0;
uint16_t sensitivity = 0;

// таймеры
uint32_t timer_cnt = 0; // опрос RadSens раз в 1с
uint32_t timer_oled = 0; // обновление OLED

// ====================== Переменные порогов ======================
bool GetDataThreshold_1;
float DoseThreshold_1 = 10000;
float DoseRateThreshold_1 = 30;
bool DoseRateSoudTh_1;
bool DoseRateVibroTh_1;
bool DoseSoudTh_1;
bool DoseVibroTh_1;

bool GetDataThreshold_2;
float DoseThreshold_2 = 1000000;
float DoseRateThreshold_2 = 100;
bool DoseRateSoudTh_2;
bool DoseRateVibroTh_2;
bool DoseSoudTh_2;
bool DoseVibroTh_2;

bool GetDataThreshold_3;
float DoseThreshold_3 = 100000000;
float DoseRateThreshold_3 = 100000;
bool DoseRateSoudTh_3;
bool DoseRateVibroTh_3;
bool DoseSoudTh_3;
bool DoseVibroTh_3;

// ====================== Вспомогательные функции (для BLE-thresholds) ======================
float getFloat(uint8_t* pData, uint8_t startIdx) {
  union {
    float val;
    uint8_t b[4];
  } float_bytes_u;
  for (uint8_t i = 0; i < 4; i++) {
    float_bytes_u.b[i] = pData[i + startIdx];
  }
  return float_bytes_u.val;
}

// ====================== BLE колбэки ======================
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    oled.clear();
    connstat = "Connected";
  };
  void onDisconnect(BLEServer* pServer){
    deviceConnected = false;
    oled.clear();
    connstat = "Disconnected";
    pServer->getAdvertising()->start();
  };
};

// Threshold callbacks
class ThresholdCallbacks_1: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *ThresholdCharacteristics_1) {
      std::string rxValue = ThresholdCharacteristics_1->getValue();
      uint8_t pData [10];
      for (int i = 0; i < rxValue.length() && i < 10; i++) {
        pData[i] = rxValue[i];
      }
      DoseThreshold_1 = getFloat(pData, 0) * 100000; //доза в мкр
      DoseRateThreshold_1 = getFloat(pData, 4) * 100; // мощность дози в мкр/ч
      switch (pData[9]) {  //флаги вибро/звука
        case 0:
          DoseRateSoudTh_1 = 0;
          DoseRateVibroTh_1 = 0;
          DoseSoudTh_1 = 0;
          DoseVibroTh_1 = 0;
          break;
        case 34:
          DoseRateSoudTh_1 = 1;
          DoseRateVibroTh_1 = 1;
          DoseSoudTh_1 = 1;
          DoseVibroTh_1 = 1;
          break;
        case 16:
          DoseRateSoudTh_1 = 1;
          DoseRateVibroTh_1 = 0;
          DoseSoudTh_1 = 0;
          DoseVibroTh_1 = 0;
          break;
        case 32:
          DoseRateSoudTh_1 = 1;
          DoseRateVibroTh_1 = 1;
          DoseSoudTh_1 = 0;
          DoseVibroTh_1 = 0;
          break;
        case 1:
          DoseRateSoudTh_1 = 0;
          DoseRateVibroTh_1 = 0;
          DoseSoudTh_1 = 1;
          DoseVibroTh_1 = 0;
          break;
        case 2:
          DoseRateSoudTh_1 = 0;
          DoseRateVibroTh_1 = 0;
          DoseSoudTh_1 = 1;
          DoseVibroTh_1 = 1;
          break;
        case 33:
          DoseRateSoudTh_1 = 1;
          DoseRateVibroTh_1 = 1;
          DoseSoudTh_1 = 1;
          DoseVibroTh_1 = 0;
          break;
        case 18:
          DoseRateSoudTh_1 = 1;
          DoseRateVibroTh_1 = 0;
          DoseSoudTh_1 = 1;
          DoseVibroTh_1 = 1;
          break;
        case 17:
          DoseRateSoudTh_1 = 1;
          DoseRateVibroTh_1 = 0;
          DoseSoudTh_1 = 1;
          DoseVibroTh_1 = 0;
          break;
      }
      GetDataThreshold_1 = 1;
    }
};
class ThresholdCallbacks_2: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *ThresholdCharacteristics_2) {
      std::string rxValue = ThresholdCharacteristics_2->getValue();
      uint8_t pData [10];
      for (int i = 0; i < rxValue.length() && i < 10; i++) {
        pData[i] = rxValue[i];
      }
      DoseThreshold_2 = getFloat(pData, 0) * 100000; //доза в мкр
      DoseRateThreshold_2 = getFloat(pData, 4) * 100; // мощность дози в мкр/ч
      switch (pData[9]) {
        case 0:
          DoseRateSoudTh_2 = 0;
          DoseRateVibroTh_2 = 0;
          DoseSoudTh_2 = 0;
          DoseVibroTh_2 = 0;
          break;
        case 34:
          DoseRateSoudTh_2 = 1;
          DoseRateVibroTh_2 = 1;
          DoseSoudTh_2 = 1;
          DoseVibroTh_2 = 1;
          break;
        case 16:
          DoseRateSoudTh_2 = 1;
          DoseRateVibroTh_2 = 0;
          DoseSoudTh_2 = 0;
          DoseVibroTh_2 = 0;
          break;
        case 32:
          DoseRateSoudTh_2 = 1;
          DoseRateVibroTh_2 = 1;
          DoseSoudTh_2 = 0;
          DoseVibroTh_2 = 0;
          break;
        case 1:
          DoseRateSoudTh_2 = 0;
          DoseRateVibroTh_2 = 0;
          DoseSoudTh_2 = 1;
          DoseVibroTh_2 = 0;
          break;
        case 2:
          DoseRateSoudTh_2 = 0;
          DoseRateVibroTh_2 = 0;
          DoseSoudTh_2 = 1;
          DoseVibroTh_2 = 1;
          break;
        case 33:
          DoseRateSoudTh_2 = 1;
          DoseRateVibroTh_2 = 1;
          DoseSoudTh_2 = 1;
          DoseVibroTh_2 = 0;
          break;
        case 18:
          DoseRateSoudTh_2 = 1;
          DoseRateVibroTh_2 = 0;
          DoseSoudTh_2 = 1;
          DoseVibroTh_2 = 1;
          break;
        case 17:
          DoseRateSoudTh_2 = 1;
          DoseRateVibroTh_2 = 0;
          DoseSoudTh_2 = 1;
          DoseVibroTh_2 = 0;
          break;
      }
      GetDataThreshold_2 = 1;
    }
};
class ThresholdCallbacks_3: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *ThresholdCharacteristics_3) {
      std::string rxValue = ThresholdCharacteristics_3->getValue();
      uint8_t pData [10];
      for (int i = 0; i < rxValue.length() && i < 10; i++) {
        pData[i] = rxValue[i];
      }
      DoseThreshold_3 = getFloat(pData, 0) * 100000; //доза в мкр
      DoseRateThreshold_3 = getFloat(pData, 4) * 100; // мощность дози в мкр/ч
      switch (pData[9]) {
        case 0:
          DoseRateSoudTh_3 = 0;
          DoseRateVibroTh_3 = 0;
          DoseSoudTh_3 = 0;
          DoseVibroTh_3 = 0;
          break;
        case 34:
          DoseRateSoudTh_3 = 1;
          DoseRateVibroTh_3 = 1;
          DoseSoudTh_3 = 1;
          DoseVibroTh_3 = 1;
          break;
        case 16:
          DoseRateSoudTh_3 = 1;
          DoseRateVibroTh_3 = 0;
          DoseSoudTh_3 = 0;
          DoseVibroTh_3 = 0;
          break;
        case 32:
          DoseRateSoudTh_3 = 1;
          DoseRateVibroTh_3 = 1;
          DoseSoudTh_3 = 0;
          DoseVibroTh_3 = 0;
          break;
        case 1:
          DoseRateSoudTh_3 = 0;
          DoseRateVibroTh_3 = 0;
          DoseSoudTh_3 = 1;
          DoseVibroTh_3 = 0;
          break;
        case 2:
          DoseRateSoudTh_3 = 0;
          DoseRateVibroTh_3 = 0;
          DoseSoudTh_3 = 1;
          DoseVibroTh_3 = 1;
          break;
        case 33:
          DoseRateSoudTh_3 = 1;
          DoseRateVibroTh_3 = 1;
          DoseSoudTh_3 = 1;
          DoseVibroTh_3 = 0;
          break;
        case 18:
          DoseRateSoudTh_3 = 1;
          DoseRateVibroTh_3 = 0;
          DoseSoudTh_3 = 1;
          DoseVibroTh_3 = 1;
          break;
        case 17:
          DoseRateSoudTh_3 = 1;
          DoseRateVibroTh_3 = 0;
          DoseSoudTh_3 = 1;
          DoseVibroTh_3 = 0;
          break;
      }
      GetDataThreshold_3 = 1;
    }
  };
class SettingsCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *SettingsCharacteristics) {
      std::string rxValue = SettingsCharacteristics->getValue();
      uint8_t pData [8];
      for (int i = 0; i < rxValue.length() && i < 8; i++) {
        pData[i] = rxValue[i];
      }
      if (pData[0] == 222 && pData[1] == 0) { Serial.println("RESET DOSE"); }
      if (pData[0] == 224 && pData[1] == 2) { Serial.println("slow"); }
      if (pData[0] == 224 && pData[1] == 1) { Serial.println("medium"); }
      if (pData[0] == 224 && pData[1] == 0) { Serial.println("fast"); }
      if (pData[0] == 226 && pData[1] == 1) { Serial.println("accelerometr ON"); }
      if (pData[0] == 226 && pData[1] == 0) { Serial.println("accelerometr OFF"); }
      if (pData[0] == 255 && pData[1] == 172) { Serial.println("power OFF"); }
      if (pData[0] == 170 && pData[1] == 0) { Serial.println("find device"); }
      if (pData[0] == 20 && pData[1] == 18) { Serial.println("reconnection sounds on"); }
      if (pData[0] == 19 && pData[1] == 18) { Serial.println("reconnection sounds off"); }
      if (pData[0] == 8 && pData[1] == 0) { Serial.println("No sound"); }
      if (pData[0] == 1 && pData[1] == 1) { Serial.println("Clicks"); }
      if (pData[0] == 1 && pData[1] == 20) { Serial.println("Beeps"); }
      if (pData[0] == 225 && pData[1] == 1) { Serial.println("1/10"); }
      if (pData[0] == 225 && pData[1] == 0) { Serial.println("1/1"); }
    }
};

// ====================== Функция отправки пакета дозы ======================
void BleSentDoseData (float RAD, float DOSERAD, float PULSE2, uint8_t battery, int8_t temperature) {
  uint8_t date[13];
  if (battery > 100) {                      // индикатор зарядки акб
    *(uint8_t*)(&date[0]) = 253;
  } else {
    *(uint8_t*)(&date[0]) = 0;
  }
  *(float*)(&date[1]) = DOSERAD / 100000.0; // доза (с масштабом)
  *(float*)(&date[5]) = RAD / 100.0;        // мощность дозы (с масштабом)
  uint16_t PULSE = (uint16_t)(PULSE2 * 4);  // костыль, как в оригинале
  *(uint16_t*)(&date[9]) = PULSE / 2;       // CPS
  *(uint8_t*)(&date[11]) = battery;         // процент заряда батареи (0..100)
  *(int8_t*)(&date[12]) = temperature;      // температура

  DoseCharacteristics.setValue(date, 13);
  DoseCharacteristics.notify();
}

// ====================== Отправка порогов (из оригинала) ======================
void SentDataThreshold_1() {
  uint8_t date[10];
  *(float*)(&date[0]) = DoseThreshold_1 / 100000;   //  доза в мкр
  *(float*)(&date[4]) = DoseRateThreshold_1 / 100;  // мощность дози в мкр/ч
  if (DoseRateSoudTh_1 == 0 && DoseRateVibroTh_1 == 0 && DoseSoudTh_1 == 0 && DoseVibroTh_1 == 0) { date[9] = 0; }
  if (DoseRateSoudTh_1 == 1 && DoseRateVibroTh_1 == 1 && DoseSoudTh_1 == 1 && DoseVibroTh_1 == 1) { date[9] = 34; }
  if (DoseRateSoudTh_1 == 1 && DoseRateVibroTh_1 == 0 && DoseSoudTh_1 == 0 && DoseVibroTh_1 == 0) { date[9] = 16; }
  if (DoseRateSoudTh_1 == 1 && DoseRateVibroTh_1 == 1 && DoseSoudTh_1 == 0 && DoseVibroTh_1 == 0) { date[9] = 32; }
  if (DoseRateSoudTh_1 == 0 && DoseRateVibroTh_1 == 0 && DoseSoudTh_1 == 1 && DoseVibroTh_1 == 0) { date[9] = 1; }
  if (DoseRateSoudTh_1 == 0 && DoseRateVibroTh_1 == 0 && DoseSoudTh_1 == 1 && DoseVibroTh_1 == 1) { date[9] = 2; }
  if (DoseRateSoudTh_1 == 1 && DoseRateVibroTh_1 == 1 && DoseSoudTh_1 == 1 && DoseVibroTh_1 == 0) { date[9] = 33; }
  if (DoseRateSoudTh_1 == 1 && DoseRateVibroTh_1 == 0 && DoseSoudTh_1 == 1 && DoseVibroTh_1 == 1) { date[9] = 18; }
  if (DoseRateSoudTh_1 == 1 && DoseRateVibroTh_1 == 0 && DoseSoudTh_1 == 1 && DoseVibroTh_1 == 0) { date[9] = 17; }

  ThresholdCharacteristics_1.setValue(date, 10);
  ThresholdCharacteristics_1.notify();
}
void SentDataThreshold_2() {
  uint8_t date[10];
  *(float*)(&date[0]) = DoseThreshold_2 / 100000;   //  доза в мкр
  *(float*)(&date[4]) = DoseRateThreshold_2 / 100;  // мощность дозы в мкр/ч
  if (DoseRateSoudTh_2 == 0 && DoseRateVibroTh_2 == 0 && DoseSoudTh_2 == 0 && DoseVibroTh_2 == 0) { date[9] = 0; }
  if (DoseRateSoudTh_2 == 1 && DoseRateVibroTh_2 == 1 && DoseSoudTh_2 == 1 && DoseVibroTh_2 == 1) { date[9] = 34; }
  if (DoseRateSoudTh_2 == 1 && DoseRateVibroTh_2 == 0 && DoseSoudTh_2 == 0 && DoseVibroTh_2 == 0) { date[9] = 16; }
  if (DoseRateSoudTh_2 == 1 && DoseRateVibroTh_2 == 1 && DoseSoudTh_2 == 0 && DoseVibroTh_2 == 0) { date[9] = 32; }
  if (DoseRateSoudTh_2 == 0 && DoseRateVibroTh_2 == 0 && DoseSoudTh_2 == 1 && DoseVibroTh_2 == 0) { date[9] = 1; }
  if (DoseRateSoudTh_2 == 0 && DoseRateVibroTh_2 == 0 && DoseSoudTh_2 == 1 && DoseVibroTh_2 == 1) { date[9] = 2; }
  if (DoseRateSoudTh_2 == 1 && DoseRateVibroTh_2 == 1 && DoseSoudTh_2 == 1 && DoseVibroTh_2 == 0) { date[9] = 33; }
  if (DoseRateSoudTh_2 == 1 && DoseRateVibroTh_2 == 0 && DoseSoudTh_2 == 1 && DoseVibroTh_2 == 1) { date[9] = 18; }
  if (DoseRateSoudTh_2 == 1 && DoseRateVibroTh_2 == 0 && DoseSoudTh_2 == 1 && DoseVibroTh_2 == 0) { date[9] = 17; }

  ThresholdCharacteristics_2.setValue(date, 10);
  ThresholdCharacteristics_2.notify();
}
void SentDataThreshold_3() {
  uint8_t date[10];
  *(float*)(&date[0]) = DoseThreshold_3 / 100000;   //  доза в мкр
  *(float*)(&date[4]) = DoseRateThreshold_3 / 100;  // мощность дозы в мкр/ч
  if (DoseRateSoudTh_3 == 0 && DoseRateVibroTh_3 == 0 && DoseSoudTh_3 == 0 && DoseVibroTh_3 == 0) { date[9] = 0; }
  if (DoseRateSoudTh_3 == 1 && DoseRateVibroTh_3 == 1 && DoseSoudTh_3 == 1 && DoseVibroTh_3 == 1) { date[9] = 34; }
  if (DoseRateSoudTh_3 == 1 && DoseRateVibroTh_3 == 0 && DoseSoudTh_3 == 0 && DoseVibroTh_3 == 0) { date[9] = 16; }
  if (DoseRateSoudTh_3 == 1 && DoseRateVibroTh_3 == 1 && DoseSoudTh_3 == 0 && DoseVibroTh_3 == 0) { date[9] = 32; }
  if (DoseRateSoudTh_3 == 0 && DoseRateVibroTh_3 == 0 && DoseSoudTh_3 == 1 && DoseVibroTh_3 == 0) { date[9] = 1; }
  if (DoseRateSoudTh_3 == 0 && DoseRateVibroTh_3 == 0 && DoseSoudTh_3 == 1 && DoseVibroTh_3 == 1) { date[9] = 2; }
  if (DoseRateSoudTh_3 == 1 && DoseRateVibroTh_3 == 1 && DoseSoudTh_3 == 1 && DoseVibroTh_3 == 0) { date[9] = 33; }
  if (DoseRateSoudTh_3 == 1 && DoseRateVibroTh_3 == 0 && DoseSoudTh_3 == 1 && DoseVibroTh_3 == 1) { date[9] = 18; }
  if (DoseRateSoudTh_3 == 1 && DoseRateVibroTh_3 == 0 && DoseSoudTh_3 == 1 && DoseVibroTh_3 == 0) { date[9] = 17; }

  ThresholdCharacteristics_3.setValue(date, 10);
  ThresholdCharacteristics_3.notify();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // RadSens init
  radSens.init();
  pulsesPrev = radSens.getNumberOfPulses();
  radSens.setSensitivity(378); // твоя настройка трубки, подбирается по параметрам

  // BLE init
  BLEDevice::init("AtomFast");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *BleService = pServer->createService(SERVICE_UUID);

  BleService->addCharacteristic(&DoseCharacteristics);
  DoseCharacteristics.addDescriptor(new BLE2902());

  BleService->addCharacteristic(&ThresholdCharacteristics_1);
  ThresholdCharacteristics_1.setCallbacks(new ThresholdCallbacks_1());

  BleService->addCharacteristic(&ThresholdCharacteristics_2);
  ThresholdCharacteristics_2.setCallbacks(new ThresholdCallbacks_2());

  BleService->addCharacteristic(&ThresholdCharacteristics_3);
  ThresholdCharacteristics_3.setCallbacks(new ThresholdCallbacks_3());

  BleService->addCharacteristic(&SettingsCharacteristics);
  SettingsCharacteristics.setCallbacks(new SettingsCallbacks());

  BleService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLE started, waiting for connection...");
  connstat = "Waiting to connect...";

  // PWM setup
  ledcSetup(1, 500, 8);

  // OLED init & splash
  oled.init();
  oled.flipV(1);
  oled.flipH(1);
  oled.clear();
  oled.setScale(1);
  oled.setCursor(0, 0);
  oled.print("RadSens_Proto_2.1");
  oled.setCursor(0, 2);
  oled.print("CG RadSens i2c v2.7");
  oled.setCursor(0, 4);
  oled.print("ESP32 + cg_charger_ip");
  oled.setCursor(0, 6);
  oled.print("Alexander Molodkin");
  delay(5000);
  oled.clear();

  // Если ты хочешь, чтобы radSens.init() выполнялся после заставки — можно раскомментировать
  // radSens.init();
  // pulsesPrev = radSens.getNumberOfPulses();
}

void loop () {
  // опрос RadSens каждую секунду
  if (millis() - timer_cnt > 1000) {
    timer_cnt = millis();
    dynval = radSens.getRadIntensyDynamic();
    statval = radSens.getRadIntensyStatic();
    impval = radSens.getNumberOfPulses();
    verval = radSens.getFirmwareVersion();
    sensitivity = radSens.getSensitivity();
    radSens.setSensitivity(378); // чувствительность
  }

  if (deviceConnected) {
    // считаем CPS за секунду:
    uint32_t cps = 0;
    if (impval >= pulsesPrev) {
      cps = impval - pulsesPrev;
    } else {
      cps = impval;
    }
    pulsesPrev = impval;

    // читаем состояние батареи
    bool usb = IP5306_GetPowerSource(); // 1 = VIN(USB), 0 = BAT
    bool full = IP5306_GetBatteryFull();
    uint8_t leds = IP5306_GetLevelLeds();
    uint8_t bat_percent = IP5306_LEDS2PCT(leds); // 0..100 (шаги 25%)

    // датчика температуры нет - ставим 0
    int8_t temperature = 0;

    // Отправляем пакет на приложение
    BleSentDoseData(dynval, statval, (float)cps, bat_percent, temperature);

    // Отправляем текущие пороги
    SentDataThreshold_1();
    SentDataThreshold_2();
    SentDataThreshold_3();
  }

  // Обновление OLED каждую секунду
  if (millis() - timer_oled > 1000) {
    timer_oled = millis();

    String dynint = "Дин: ";
    dynint += dynval;
    String statint = "Ст: ";
    statint += statval;
    String nimp = "Имп: ";
    nimp += impval;
    String Ver = "FW:";
    Ver += verval;
    String sens = "S:";
    sens += sensitivity;

    oled.setCursor(0, 2);
    oled.print(dynint);
    oled.setCursor(64, 2);
    oled.print(statint);
    oled.setCursor(68, 1);
    oled.print(nimp);
    oled.setCursor(0, 1);
    oled.print(Ver);
    oled.setCursor(34, 1);
    oled.print(sens);
    oled.setCursor(0, 0);

    bool usb = IP5306_GetPowerSource();
    bool full = IP5306_GetBatteryFull();
    uint8_t leds = IP5306_GetLevelLeds();
    oled.printf("sc:%s,st:%s,bt:%u%%\n", usb?"U":"B", full?"full":(usb?"CHG":"DCHG"), IP5306_LEDS2PCT(leds));

    oled.setCursor(0, 7);
    oled.print(connstat);
  }

  // Обработка приходящих порогов и вывод в Serial
  if (GetDataThreshold_1 == 1) {
    GetDataThreshold_1 = 0;
    Serial.println();
    Serial.print("DoseRateThreshold_1 = ");
    Serial.print(DoseRateThreshold_1);
    Serial.print("    ");
    Serial.print(DoseRateSoudTh_1);
    Serial.print("    ");
    Serial.print(DoseRateVibroTh_1);
    Serial.println();
    Serial.print("DoseThreshold_1 = ");
    Serial.print(DoseThreshold_1);
    Serial.print("    ");
    Serial.print(DoseSoudTh_1);
    Serial.print("    ");
    Serial.print(DoseVibroTh_1);
    Serial.println();
  }
  if (GetDataThreshold_2 == 1) {
    GetDataThreshold_2 = 0;
    Serial.println();
    Serial.print("DoseRateThreshold_2 = ");
    Serial.print(DoseRateThreshold_2);
    Serial.print("    ");
    Serial.print(DoseRateSoudTh_2);
    Serial.print("    ");
    Serial.print(DoseRateVibroTh_2);
    Serial.println();
    Serial.print("DoseThreshold_2 = ");
    Serial.print(DoseThreshold_2);
    Serial.print("    ");
    Serial.print(DoseSoudTh_2);
    Serial.print("    ");
    Serial.print(DoseVibroTh_2);
    Serial.println();
  }
  if (GetDataThreshold_3 == 1) {
    GetDataThreshold_3 = 0;
    Serial.println();
    Serial.print("DoseRateThreshold_3 = ");
    Serial.print(DoseRateThreshold_3);
    Serial.print("    ");
    Serial.print(DoseRateSoudTh_3);
    Serial.print("    ");
    Serial.print(DoseRateVibroTh_3);
    Serial.println();
    Serial.print("DoseThreshold_3 = ");
    Serial.print(DoseThreshold_3);
    Serial.print("    ");
    Serial.print(DoseSoudTh_3);
    Serial.print("    ");
    Serial.print(DoseVibroTh_3);
    Serial.println();
  }

  delay(1000);
}
