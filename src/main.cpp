#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <M5AtomS3.h>

#include <cmath>
#include <cstring>

#include "HLW8032.h"
#include "config.h"
#include "freqcount.h"

#define BLE_SERVICE_UUID "0000181a-0000-1000-8000-00805f9b34fb"
#define BLE_CHARACTERISTIC_VOLTAGE_UUID "00002b18-0000-1000-8000-00805f9b34fb"
#define BLE_CHARACTERISTIC_FREQ_UUID "00002be8-0000-1000-8000-00805f9b34fb"

M5GFX lcd;
M5Canvas canvas(&M5.Lcd);
FreqCountIRQ<PIN_IN_FREQ> freq_count;
HLW8032 sensor;
BLEServer *pServer                        = nullptr;
BLECharacteristic *pCharacteristicVoltage = nullptr;
BLECharacteristic *pCharacteristicFreq    = nullptr;
BLEAdvertising *pAdvertising              = nullptr;

bool deviceConnected = false;
int32_t deviceCount  = 0;

class BLECallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pBLEServer) override {
    deviceConnected = true;
    deviceCount++;
    Serial.println("Device connected");
  };

  void onDisconnect(BLEServer *pBLEServer) override {
    deviceConnected = false;
    deviceCount--;
    Serial.println("Device disconnected");
  }
};

void updateDisplay() {
  char buff[20];
  static uint8_t seq;
  bool freqIsValid = freq_count.update();
  double freq      = (double)(freq_count.get_observed_frequency() * 0.9999724008 + 0.0000109997);
  double voltage   = sensor.getEffectiveVoltage();

  if (std::isnan(freq)) {
    freq = 0.0;
  }

  if (std::isnan(voltage)) {
    voltage = 0.0;
  }

  canvas.clear(BLACK);
  canvas.setTextFont(4);

  canvas.setTextColor(GREENYELLOW, BLACK);
  dtostrf(voltage, 3, 3, buff);
  canvas.drawRightString(buff, 100, 0);
  canvas.drawString("V", 100, 0);

  if (freqIsValid) {
    canvas.setTextColor(ORANGE, BLACK);
    dtostrf(freq, 2, 3, buff);
    canvas.drawRightString(buff, 100, 20);
    canvas.drawString("Hz", 100, 20);
  } else {
    canvas.setTextColor(ORANGE, BLACK);
    canvas.drawRightString("--.---", 100, 20);
    canvas.drawString("Hz", 100, 20);
  }

  // canvas.setTextFont(2);

  if (deviceConnected) {
    canvas.setTextColor(RED, BLACK);
    ltoa(deviceCount, buff, 10);
    canvas.drawRightString(buff, 100, 40);
    canvas.drawString("CN", 100, 40);

    std::string strNotifyDataVoltage = "";
    strNotifyDataVoltage.append(reinterpret_cast<const char *>(&voltage), sizeof(double));
    pCharacteristicVoltage->setValue(strNotifyDataVoltage);
    pCharacteristicVoltage->notify();

    std::string strNotifyDataFreq = "";
    strNotifyDataFreq.append(reinterpret_cast<const char *>(&freq), sizeof(double));
    pCharacteristicFreq->setValue(strNotifyDataFreq);
    pCharacteristicFreq->notify();
  } else {
    canvas.setTextColor(RED, BLACK);
    ltoa(deviceCount, buff, 10);
    canvas.drawRightString(buff, 100, 40);
    canvas.drawString("CN", 100, 40);

    BLEAdvertisementData advertisementData = BLEAdvertisementData();
    advertisementData.setFlags(0x06);  // BR_EDR_NOT_SUPPORTED | General Discoverable Mode

    std::string strServiceData = "";
    strServiceData += (char)0x04;  // length（4 bytes）
    strServiceData += (char)0x16;  // Type 0x16: Service Data
    strServiceData += (char)0x5b;  // Service Data UUID
    strServiceData += (char)0x85;  // Service Data UUID
    strServiceData += (char)seq++;

    advertisementData.addData(strServiceData);
    pAdvertising->setAdvertisementData(advertisementData);
  }

  canvas.pushSprite(0, 0);
}

void setup() {
  auto config = M5.config();
  AtomS3.begin(config);
  Serial.begin(115200);
  M5.Lcd.init();
  BLEDevice::init("ACObservator");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BLECallbacks());
  BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
  pCharacteristicVoltage =
      pService->createCharacteristic(BLE_CHARACTERISTIC_VOLTAGE_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicVoltage->addDescriptor(new BLE2902());
  pCharacteristicFreq =
      pService->createCharacteristic(BLE_CHARACTERISTIC_FREQ_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicFreq->addDescriptor(new BLE2902());
  pService->start();
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  canvas.setColorDepth(8);
  canvas.createSprite(M5.Lcd.width(), M5.Lcd.height());

  pinMode(PIN_IN_FREQ, INPUT);
  pinMode(PIN_IN_SD, INPUT);
  pinMode(PIN_IN_PF, INPUT);

  Serial1.begin(4800, SERIAL_8E1, PIN_IN_SD);

  freq_count.begin();
  canvas.setTextSize(1);
}

void loop() {
  AtomS3.update();

  while (Serial1.available() > 0) {
    sensor.processData(Serial1.read());
  }

  updateDisplay();

  if (!deviceConnected) {
    pAdvertising->start();
    delay(1000);
    pAdvertising->stop();
  } else {
    delay(1000);
  }
}
