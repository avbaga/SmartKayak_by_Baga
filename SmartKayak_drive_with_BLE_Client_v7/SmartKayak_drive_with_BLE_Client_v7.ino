/* Smart Kayak v2
this part od code is main Kayak driver 
and BLE client for padel tenzosensors
This project is designed and definet by Alexander Bogachenko baga@mail.ru 
*/ 
//#define PRINT_IMU_TO_CONSOLE // uncoment if you want to see IMU data in serial

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

//-----------------------------------------------------------
//IMU configuration
#define I2C_SDA 21
#define I2C_SCL 22
#define INTERRUPT_PIN 23  
#include "Wire.h"
#include "I2Cdev.h"

// Barometer configuration
#include <iarduino_Pressure_BMP.h> 
iarduino_Pressure_BMP baro; 


//#include "MPU6050.h" // подключаем вместо следующей
#include "MPU6050_6Axis_MotionApps20.h"

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


 /* ========================================================================= */
// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

/* эта функция работает ниже но не работает тут
// set interrupt for MPU6050
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
*/
MPU6050 accelgyro;
int16_t kayak_ax, kayak_ay, kayak_az;
float kayak_nax, kayak_nay, kayak_naz;
int16_t kayak_gx, kayak_gy, kayak_gz;
float kayak_ngx, kayak_ngy, kayak_ngz;
int16_t kayak_temperature;
float kayak_ntemp;

#include <QMC5883LCompass.h> // ebuchi compas желательно поменять на что-то работающее
QMC5883LCompass compass;
int kayak_mx, kayak_my, kayak_mz, kayak_azimuth, kayak_bearing;
float kayak_nmx, kayak_nmy, kayak_nmz;

// Mdgwick lib
#include <MadgwickAHRS.h>
Madgwick filter;
float kayak_roll, kayak_pitch, kayak_heading;


//-----------------------------------------------------------
// BLE client setup
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp32-hal-ledc.h>
static BLEUUID serviceUUID("4b2de81d-c131-4636-8d56-83b83758f7ca"); // Specify the Service UUID of Padel BLE Server
//static BLEUUID Force_L_UUID("beb5483e-36e1-4688-b7f5-ea07361b26a8"); // Specify the LEFT FORCE Characteristic UUID of Server
//static BLEUUID Force_R_UUID("cba1d466-344c-4be3-ab3f-189f80dd7518"); // Specify the RIGHT FORCE Characteristic UUID of Server
static BLEUUID CHARACTERISTIC_IMU_UUID("d2e5bfeb-d9f8-4b75-a295-d3f4032086ea"); // Добавлено

//String sForce_L_UUID = Force_L_UUID.toString().c_str();
//String sForce_R_UUID = Force_R_UUID.toString().c_str();
String sIMU_UUID = CHARACTERISTIC_IMU_UUID.toString().c_str(); // Добавлено
BLEClient*  pClient = nullptr;

int32_t iForceL = 0;
int32_t iForceR = 0;
  
bool doConnect = false;
bool isConnected = false;
bool doScan = false;
unsigned long lastConnectionAttempt = 0;
const unsigned long CONNECTION_ATTEMPT_INTERVAL = 5000; // 5 секунд между попытками подключения

//static BLERemoteCharacteristic* pRemoteForceL_Characteristic;
//static BLERemoteCharacteristic* pRemoteForceR_Characteristic;
static BLERemoteCharacteristic* pRemoteIMU_Characteristic; // Добавлено
static BLEAdvertisedDevice* myDevice;


struct IMUData {
    int32_t LoadCell_L, LoadCell_R;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int mx, my, mz;
    int azimuth;
    float roll, pitch, heading;
  } imuDataStruct;

static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                            uint8_t* pData, size_t length, bool isNotify)
{
  String sBLERemoteCharacteristic = pBLERemoteCharacteristic->getUUID().toString().c_str();
  if (sIMU_UUID == sBLERemoteCharacteristic) {
  //  if (length == sizeof(IMUData)) {
      memcpy(&imuDataStruct, pData, sizeof(IMUData));
      Serial.println("Получены данные с Весла:");
      iForceL = imuDataStruct.LoadCell_L;
      iForceR = imuDataStruct.LoadCell_R;
      Serial.print("LoadCell_L: "); Serial.print(imuDataStruct.LoadCell_L);
      Serial.print("LoadCell_R: "); Serial.print(imuDataStruct.LoadCell_R);
      Serial.print("ax: "); Serial.print(imuDataStruct.ax);
      Serial.print(", ay: "); Serial.print(imuDataStruct.ay);
      Serial.print(", az: "); Serial.println(imuDataStruct.az);
      Serial.print(", gx: "); Serial.println(imuDataStruct.gx);
      Serial.print(", gy: "); Serial.println(imuDataStruct.gy);
      Serial.print(", gz: "); Serial.println(imuDataStruct.gz);
      // Добавьте вывод остальных данных по желанию
    //}
  }
/*
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
  */
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

/*
  // Obtain a reference to the LEFT FORCE characteristic in the service of the remote BLE server 
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
  
  // Read the value of the Duty characteristic 
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

  // Obtain a reference to the RIGHT FORCE characteristic in the service of the remote BLE server 
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

  // Read the value of the Force characteristic 
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
*/
  // Получение характеристики IMU 
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
}

//----------------------------------------------------
// set interrupt for MPU6050
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
//------------------------------------------------------

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
//-----------------------------------------------
 // Start I2C interface 
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // 400kHz I2C clock.

  // Baro initialization 
  Serial.println("Initializing BMP180 Barometer");
  baro.begin(0);
  baro.measurement(3); // 0 - грязные данные 1-2-3 повышение качества данных

  // initializing IMU GY-87
  Serial.println("Initializing I2C devices...");
  // initialize QMC5883L compas device
  Serial.println("Initializing QMC5883L Compass");
  compass.init();
  Serial.println("Setting OffSets for Compass");
  compass.setCalibrationOffsets(50.00, 1423.00, -265.00); // заменить на механизм автоматической калибровки
  compass.setCalibrationScales(0.87, 1.06, 1.11); // заменить на механизм автоматической калибровки

  // initialize MPU6050 AccelGyro device
  Serial.println("Initializing MPU6050 Accelerometer ang Giroscope");
  accelgyro.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);      
  
  Serial.println("Setting OffSets for Accel and Gyro");
  devStatus = accelgyro.dmpInitialize();
  // [-7113,-7111] --> [-4,9]	[-803,-802] --> [-11,6]	[611,612] --> [16380,16398]	[-41,-40] --> [0,1]	[-49,-48] --> [0,1]	[-24,-23] --> [0,1]
    accelgyro.setXAccelOffset(-7113);
    accelgyro.setYAccelOffset(-803);
    accelgyro.setZAccelOffset(611);
    accelgyro.setXGyroOffset(-41);
    accelgyro.setYGyroOffset(-49);
    accelgyro.setZGyroOffset(-24);   
  
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    Serial.println("Calibrating Accel and Gyro");
    accelgyro.CalibrateAccel(6);
    accelgyro.CalibrateGyro(6);
    accelgyro.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    accelgyro.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = accelgyro.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = accelgyro.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
 Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

// это какой-то странный код для того чтобы было видно QMC5883L вместе с MPU6050
  //Bypass Mode
  Wire.beginTransmission(0x68);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();
  
// Madgwick initialization
Serial.println("Initializing Madgwick Quadrobion");
  filter.begin(25);


//-------------------------------------------------
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

  if (!dmpReady) return;
    // read a packet from FIFO
    if (accelgyro.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      #ifdef OUTPUT_READABLE_QUATERNION
        // display quaternion values in easy matrix form: w x y z
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        #ifdef PRINT_IMU_TO_CONSOLE
          Serial.print("quat\t");
          Serial.print(q.w);
          Serial.print("\t");
          Serial.print(q.x);
          Serial.print("\t");
          Serial.print(q.y);
          Serial.print("\t");
          Serial.println(q.z);
         #endif 
      #endif

      #ifdef OUTPUT_READABLE_EULER
        // display Euler angles in degrees
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        accelgyro.dmpGetEuler(euler, &q);
        #ifdef PRINT_IMU_TO_CONSOLE
          Serial.print("euler\t");
          Serial.print(euler[0] * 180/M_PI);
          Serial.print("\t");
          Serial.print(euler[1] * 180/M_PI);
          Serial.print("\t");
          Serial.println(euler[2] * 180/M_PI);
        #endif
      #endif

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        accelgyro.dmpGetGravity(&gravity, &q);
        accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #ifdef PRINT_IMU_TO_CONSOLE
          Serial.print("ypr\t");
          Serial.print(ypr[0] * 180/M_PI);
          Serial.print("\t");
          Serial.print(ypr[1] * 180/M_PI);
          Serial.print("\t");
          Serial.println(ypr[2] * 180/M_PI);
        #endif
      #endif

      #ifdef OUTPUT_READABLE_REALACCEL
        // display real acceleration, adjusted to remove gravity
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        accelgyro.dmpGetAccel(&aa, fifoBuffer);
        accelgyro.dmpGetGravity(&gravity, &q);
        accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        #ifdef PRINT_IMU_TO_CONSOLE
          Serial.print("areal\t");
          Serial.print(aaReal.x);
          Serial.print("\t");
          Serial.print(aaReal.y);
          Serial.print("\t");
          Serial.println(aaReal.z);
        #endif
      #endif

      #ifdef OUTPUT_READABLE_WORLDACCEL
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        accelgyro.dmpGetAccel(&aa, fifoBuffer);
        accelgyro.dmpGetGravity(&gravity, &q);
        accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        accelgyro.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        #ifdef PRINT_IMU_TO_CONSOLE
          Serial.print("aworld\t");
          Serial.print(aaWorld.x);
          Serial.print("\t");
          Serial.print(aaWorld.y);
          Serial.print("\t");
          Serial.println(aaWorld.z);
        #endif
      #endif    
    }  

    // IMU Reading RAW data 
    accelgyro.getMotion6(&kayak_ax, &kayak_ay, &kayak_az, &kayak_gx, &kayak_gy, &kayak_gz); 
    kayak_temperature = accelgyro.getTemperature(); 

    compass.read(); //сраный компас так нормально и не заработал
      kayak_mx = compass.getX();
      kayak_my = compass.getY();
      kayak_mz = compass.getZ();
      kayak_azimuth = compass.getAzimuth();
      kayak_bearing = compass.getBearing(kayak_azimuth);
      char myArray[3];
      compass.getDirection(myArray, kayak_azimuth);

    //barometer read - не знаю зачем он нам но раз есть я его подключил
    #ifdef PRINT_IMU_TO_CONSOLE
      if( baro.read(2) ){ // Функция read(2) читает давление в Па, температуру в °С и высоту в м.
          Serial.print((String) "Датчик BMP"+ baro.type        + ": ");
          Serial.print((String) "P = "      + baro.pressure    + " Pa, \t");
          Serial.print((String) "T = "      + baro.temperature + " *C, \t\t");
          Serial.print((String) "B = "      + baro.altitude    + " м.\r\n");
      }else{
          Serial.println("Нет ответа от датчика.");   
      }
    #endif
    // convert from raw data to gravity and degrees/second units
    kayak_nax = convertRawAcceleration(kayak_ax);
    kayak_nay = convertRawAcceleration(kayak_ay);
    kayak_naz = convertRawAcceleration(kayak_az);
    kayak_ngx = convertRawGyro(kayak_gx);
    kayak_ngy = convertRawGyro(kayak_gy);
    kayak_ngz = convertRawGyro(kayak_gz);
    kayak_ntemp = convertTemp(kayak_temperature);
    kayak_nmx = convertRawCompas(kayak_mx);
    kayak_nmy = convertRawCompas(kayak_my);
    kayak_nmz = convertRawCompas(kayak_mz);

    // update the квадробион, which computes orientation
    filter.update(kayak_ngx, kayak_ngy, kayak_ngz, kayak_nax, kayak_nay, kayak_naz, kayak_nmx, kayak_nmy, kayak_nmz);

    // выводим показатели в серийный порт
    #ifdef PRINT_IMU_TO_CONSOLE
      // print IMU values
      Serial.print("\t| acc ");
      Serial.print(kayak_nax); Serial.print(" ");
      Serial.print(kayak_nay); Serial.print(" ");
      Serial.print(kayak_naz);
      
      Serial.print("\t| gyr ");
      Serial.print(kayak_ngx); Serial.print(" ");
      Serial.print(kayak_ngy); Serial.print(" ");
      Serial.print(kayak_ngz);
      
      Serial.print("| t* ");
      Serial.print(kayak_ntemp);
      
      Serial.print(" | mag"); 
      Serial.print(kayak_nmx); Serial.print(" ");
      Serial.print(kayak_nmy); Serial.print(" ");
      Serial.print(kayak_nmz); Serial.print(" ");
      Serial.print(" Azimuth: ");	Serial.print(kayak_azimuth);
      Serial.print(" Bearing: ");	Serial.print(kayak_bearing);
      Serial.print(" Direction: "); Serial.print(myArray[0]); Serial.print(myArray[1]); Serial.print(myArray[2]);
      Serial.println();

      // print the quadrobion heading, pitch and roll
      kayak_roll = filter.getRoll();
      kayak_pitch = filter.getPitch();
      kayak_heading = filter.getYaw();
      Serial.print("Madgwick Q*: "); 
      Serial.print(kayak_heading);
      Serial.print(" ");
      Serial.print(kayak_pitch);
      Serial.print(" ");
      Serial.println(kayak_roll);
    #endif
//  delay(500);
}


float convertRawAcceleration(int16_t aRaw) {
  float a = aRaw  / 16384.0;
  return a;
}

float convertRawGyro(int16_t gRaw) {
  float g = gRaw / 16.4;
  return g;
}

float convertTemp(int16_t temper) {
  float t = (temper - 531) / 340.0 + 35.0;
  return t;
};

float convertRawCompas(int mag) {
  float m = mag / 3000.0;
  return m;
}