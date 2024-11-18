/* Smart Kayak v2
this part od code is main Kayak driver 
and BLE client for padel tenzosensors
This project is designed and definet by Alexander Bogachenko baga@mail.ru 
*/ 
// Define pins
#define BUTTON_PIN    18 // ESP32 pin GPIO18, which connected to power mode changing button
#define MOTOR_PIN     16 // PWM output for motor driver
#define REVERSE_PIN   19 // output for reverce direction PIN
#define LED_PIN       25 // PWM MOTOR Duty LED 
#define LOW_LED_PIN   33 // Green LED
#define MED_LED_PIN   27 // Blue LED
#define HIGH_LED_PIN  26 // Red LED


// Define PWM output and duty Cycle mode
#define PWM1_Ch    0
#define PWM1_Res   8
#define PWM1_Freq  1000
#define PWM1_HIGH  3.0 
#define PWM1_MED   2.0
#define PWM1_LOW   1.0

// define initial power state, mode and force threshold
const int FORCE_THRESHOLD = 20; // минимальный уровень чувствительности тензодатчиков
const int MAX_FORCE = 100; // максимальное усилие на весле
int PWM1_DutyCycle = 0;
int Padel_mode = 3; // Starting from LOW mode

// BLE client setup
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp32-hal-ledc.h>
static BLEUUID serviceUUID("4b2de81d-c131-4636-8d56-83b83758f7ca"); // Specify the Service UUID of Padel BLE Server
static BLEUUID Force_L_UUID("beb5483e-36e1-4688-b7f5-ea07361b26a8"); // Specify the LEFT FORCE Characteristic UUID of Server
static BLEUUID Force_R_UUID("cba1d466-344c-4be3-ab3f-189f80dd7518"); // Specify the RIGHT FORCE Characteristic UUID of Server
static BLEUUID CHARACTERISTIC_IMU_UUID("d2e5bfeb-d9f8-4b75-a295-d3f4032086ea"); // Добавлено

String sForce_L_UUID = Force_L_UUID.toString().c_str();
String sForce_R_UUID = Force_R_UUID.toString().c_str();
String sIMU_UUID = CHARACTERISTIC_IMU_UUID.toString().c_str(); // Добавлено
BLEClient*  pClient = nullptr;

int32_t iForceL = 0;
int32_t iForceR = 0;
  
bool doConnect = false;
bool isConnected = false;
bool doScan = false;
unsigned long lastConnectionAttempt = 0;
const unsigned long CONNECTION_ATTEMPT_INTERVAL = 5000; // 5 секунд между попытками подключения

static BLERemoteCharacteristic* pRemoteForceL_Characteristic;
static BLERemoteCharacteristic* pRemoteForceR_Characteristic;
static BLERemoteCharacteristic* pRemoteIMU_Characteristic; // Добавлено
static BLEAdvertisedDevice* myDevice;

struct IMUData {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int mx, my, mz;
  int azimuth;
} imuDataStruct;

static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                            uint8_t* pData, size_t length, bool isNotify)
{
  String sBLERemoteCharacteristic = pBLERemoteCharacteristic->getUUID().toString().c_str();

  if (sForce_L_UUID == sBLERemoteCharacteristic) {
    iForceL = (int32_t)(*((int32_t*)pData));
    Serial.print("Left: ");
    Serial.println((int)iForceL, DEC);
  }
  else if (sForce_R_UUID == sBLERemoteCharacteristic) {
    iForceR = (int32_t)(*((int32_t*)pData));
    Serial.print("Right: ");
    Serial.println((int)iForceR, DEC);
  }
  else if (sIMU_UUID == sBLERemoteCharacteristic) {
  //  if (length == sizeof(IMUData)) {
      memcpy(&imuDataStruct, pData, sizeof(IMUData));
      Serial.println("Получены данные IMU:");
      Serial.print("ax: "); Serial.print(imuDataStruct.ax);
      Serial.print(", ay: "); Serial.print(imuDataStruct.ay);
      Serial.print(", az: "); Serial.println(imuDataStruct.az);
      // Добавьте вывод остальных данных по желанию
    //}
  }
}

void scanCompletedCallback(BLEScanResults scanResults) {
  if (myDevice) {
    doConnect = true;
  } else {
    Serial.println("Paddle not found. Retry in 5 seconds.");
  }
}

class MyClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient* pclient)
  {
    Serial.println("Connected to a paddle");
    isConnected=true;
  }

  void onDisconnect(BLEClient* pclient)
  {
    isConnected = false;
    Serial.println("Disconnected from a paddle");
    if (pClient) {
      delete pClient;
      pClient = nullptr;
    }
  }
};

/* Start connection to the BLE Server */
bool connectToServer()
{
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());
    
  pClient  = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

    /* Connect to the remote BLE Server */
  if (!pClient->connect(myDevice)) {
    Serial.println(" - Failed to connect to server");
    delete pClient;
    pClient = nullptr;
    return false;
  }
  Serial.println(" - Connected to server");

    /* Obtain a reference to the service we are after in the remote BLE server */
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr)
  {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    if (pClient) {
      delete pClient;
      pClient = nullptr;
    }
    return false;
  }
  Serial.println(" - Found our service");


  /* Obtain a reference to the LEFT FORCE characteristic in the service of the remote BLE server */
  pRemoteForceL_Characteristic = pRemoteService->getCharacteristic(Force_L_UUID);
  if (pRemoteForceL_Characteristic == nullptr)
  {
    Serial.print("Failed to find LEFT FORCE SENSOR: ");
    Serial.println(Force_L_UUID.toString().c_str());
    pClient->disconnect();
    if (pClient) {
      delete pClient;
      pClient = nullptr;
    }
    return false;
  }
  Serial.println(" - Found LEFT FORCE SENSOR");
  
  /* Read the value of the Duty characteristic */
  if(pRemoteForceL_Characteristic->canRead())
  {
    std::string value = pRemoteForceL_Characteristic->readValue();
    Serial.print("LEFT FORCE value was: ");
    Serial.println(value.c_str());
  }

  if(pRemoteForceL_Characteristic->canNotify())
  {
    pRemoteForceL_Characteristic->registerForNotify(notifyCallback);
  }

  /* Obtain a reference to the RIGHT FORCE characteristic in the service of the remote BLE server */
  pRemoteForceR_Characteristic = pRemoteService->getCharacteristic(Force_R_UUID);
  if (pRemoteForceR_Characteristic == nullptr)
  {
    Serial.print("Failed to find RIGHT FORCE SENSOR: ");
    Serial.println(Force_R_UUID.toString().c_str());
    pClient->disconnect();
    if (pClient) {
      delete pClient;
      pClient = nullptr;
    }
    return false;
  }
  Serial.println(" - Found RIGHT FORCE SENSOR");

  /* Read the value of the Force characteristic */
  if(pRemoteForceR_Characteristic->canRead())
  {
    std::string value = pRemoteForceR_Characteristic->readValue();
    Serial.print("RIGHT FORCE value was: ");
    Serial.println(value.c_str());
  }

  if(pRemoteForceR_Characteristic->canNotify())
  {
    pRemoteForceR_Characteristic->registerForNotify(notifyCallback);
  }

  /* Получение характеристики IMU */
  pRemoteIMU_Characteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_IMU_UUID);
  if (pRemoteIMU_Characteristic == nullptr)
  {
    Serial.print("Не удалось найти характеристику IMU: ");
    Serial.println(CHARACTERISTIC_IMU_UUID.toString().c_str());
    pClient->disconnect();
    if (pClient) {
      delete pClient;
      pClient = nullptr;
    }
    return false;
  }
  Serial.println(" - Найдена характеристика IMU");

  if(pRemoteIMU_Characteristic->canNotify())
  {
    pRemoteIMU_Characteristic->registerForNotify(notifyCallback);
  }

  isConnected = true;
  return true;
}
/* Scan for BLE servers and find the first one that advertises the service we are looking for. */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks
{
 /* Called for each advertising BLE server. */
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    /* We have found a device, let us now see if it contains the service we are looking for. */
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID))
    {
      if (!doConnect&&!myDevice){
        BLEDevice::getScan()->stop();
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
        doConnect = true;
        Serial.println("Paddle found");
      }
    }
  }
};
// end of BLE initiation

// button handling
volatile bool buttonPressed = false;

void handleButtonPress() {
  static unsigned long lastDebounceTime = 0;
  unsigned long debounceDelay = 200; // Задержка для устранения дребезга контактов

  if ((millis() - lastDebounceTime) > debounceDelay) {
    switch (Padel_mode) {
      case 1:
        Padel_mode = 3; // Установить режим LOW
        digitalWrite(LOW_LED_PIN, HIGH);
        digitalWrite(MED_LED_PIN, LOW);
        digitalWrite(HIGH_LED_PIN, LOW);
        Serial.println("Переключение режима весла на LOW");
        break;
      case 2:
        Padel_mode = 1; // Установить режим POWER
        digitalWrite(LOW_LED_PIN, LOW);
        digitalWrite(MED_LED_PIN, LOW);
        digitalWrite(HIGH_LED_PIN, HIGH);
        Serial.println("Переключение режима весла на HIGH");
        break;
      case 3:
        Padel_mode = 2; // Установить режим MEDIUM
        digitalWrite(LOW_LED_PIN, LOW);
        digitalWrite(MED_LED_PIN, HIGH);
        digitalWrite(HIGH_LED_PIN, LOW);
        Serial.println("Переключение режима весла на MEDIUM");
        break;
    }
    lastDebounceTime = millis();
  }
  buttonPressed = false;
}

void buttonInterrupt() {
  buttonPressed = true;
};

// Обновленная функция управления мотором
void driveMotor(int32_t iForceR, int32_t iForceL) {
  int32_t absForceR = abs(iForceR);
  int32_t absForceL = abs(iForceL);
  int32_t maxForce = max(absForceR, absForceL);
  int32_t iForceFull = (absForceR > absForceL) ? iForceR : iForceL;
  
  if (maxForce > FORCE_THRESHOLD) {
    float modeMultiplier;
    switch (Padel_mode) {
      case 1: modeMultiplier = PWM1_HIGH; break;
      case 2: modeMultiplier = PWM1_MED; break;
      case 3: modeMultiplier = PWM1_LOW; break;
      default: modeMultiplier = PWM1_LOW;
    }
    
    int32_t adjustedForce = maxForce * modeMultiplier;
    PWM1_DutyCycle = (map(adjustedForce, 0, MAX_FORCE, 0, 255) > 255) ? 255 : map(adjustedForce, 0, MAX_FORCE, 0, 255);
    
    bool isReverse = (iForceFull < 0);
    digitalWrite(REVERSE_PIN, !isReverse);
    ledcWrite(PWM1_Ch, PWM1_DutyCycle);
    
    Serial.print("Full Force: ");
    Serial.print(iForceFull);
    Serial.print("\t | adjusted to ");
    Serial.print(isReverse ? "- " : "+ ");
    Serial.print(PWM1_DutyCycle);
    Serial.print("\t | With multiplier: ");
    Serial.println(modeMultiplier);
  } else {
    digitalWrite(REVERSE_PIN, HIGH);
    ledcWrite(PWM1_Ch, 0);
    // Serial.println("No pressure detected");
  }
};

void setup() {
  Serial.begin(115200);

  //BLE setup
  BLEDevice::init("ESP32-BLE-Client on Kayak");
  /* Retrieve a Scanner and set the callback we want to use to be informed when we
     have detected a new device.  Specify that we want active scanning and start the
     scan to run for 5 seconds. */
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  //pBLEScan->start(5, false);
  //pBLEScan->start(5, scanCompletedCallback);
  doScan=true;
  // End of BLE setup

  // set the ADC attenuation to 11 dB (up to ~3.3V input)
  analogSetAttenuation(ADC_11db);
  
  // set the MOTOR
  ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
  ledcAttachPin(MOTOR_PIN, PWM1_Ch);
  pinMode(REVERSE_PIN, OUTPUT);
  digitalWrite(REVERSE_PIN, HIGH);

 // set the FORCE POWER LED
  ledcAttachPin(LED_PIN, PWM1_Ch);
  
  // start sheme of indicator led
  pinMode(LOW_LED_PIN, OUTPUT);
  pinMode(MED_LED_PIN, OUTPUT);
  pinMode(HIGH_LED_PIN, OUTPUT);
  digitalWrite(LOW_LED_PIN, HIGH);
  digitalWrite(MED_LED_PIN, LOW);
  digitalWrite(HIGH_LED_PIN, LOW);


  //mode button setup
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonInterrupt, FALLING);
  Serial.println("READY, STEADY, GO!");
}

void loop() {
  // BLE part
  unsigned long currentMillis = millis();

  if (!isConnected&&!doConnect) {
    if ((currentMillis - lastConnectionAttempt >= CONNECTION_ATTEMPT_INTERVAL) || doScan) {
      lastConnectionAttempt = currentMillis;
      Serial.println("Searching for a paddle...");
      BLEDevice::getScan()->start(1, scanCompletedCallback);
    }
  }
  if (doConnect)
  {
    if (connectToServer())
    {
      Serial.println("We are now connected to the BLE Server.");
      isConnected = true;
      doConnect = false;
    } 
    else
    {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
      isConnected = false;
      doConnect = false;
      if (myDevice) {
        delete myDevice;
        myDevice = nullptr;
      }
      doScan = true;
    }
  }

  if (isConnected) 
  {
    driveMotor(iForceR, iForceL);
  };  
  
  if (buttonPressed) {
    handleButtonPress();
  }
  delay(500);
}
