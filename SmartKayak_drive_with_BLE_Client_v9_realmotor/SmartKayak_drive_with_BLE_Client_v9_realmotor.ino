/* Smart Kayak v2
this part od code is main Kayak driver 
and BLE client for padel tenzosensors
This project is designed and definet by Alexander Bogachenko baga@mail.ru 
*/ 

// Define pins
#define BUTTON_PIN    17 // Force power mode changing button
#define MOTOR_PIN     32 // PWM output for motor driver
#define DUTY_LED_PIN  25 // PWM MOTOR Duty LED 
#define LOW_LED_PIN   14 // Force mode Green LED
#define MED_LED_PIN   27 // Force mode Blue LED
#define HIGH_LED_PIN  26 // Force mode Red LED


// Define Motor PWM output and duty Cycle mode
#include <ESP32Servo.h>
Servo servo;
#define PWM1_Ch    1
#define PWM1_Res   8
#define PWM1_Freq  1000
#define PWM1_HIGH  3.0 // мультипликатор усилия для режима HIGH
#define PWM1_MED   2.0 // мультипликатор усилия для режима MED
#define PWM1_LOW   1.0 // мультипликатор усилия для режима LOW
const int32_t FORCE_THRESHOLD = 1000; // нижний порог чувствительности тензодатчиков в граммах
const int32_t MAX_FORCE = 10000; // максимальное возможное усилие на весле в граммах
int PWM1_DutyCycle = 1500; // стартовая загрузка ШИМ управления мотором
int PWM1_DutyLED = 0; // начальная яркость LED отображения усилия
int Padel_mode = 3; // Начальный режим работы LOW
long lastMotorDriveTime = 0; 
long MotorDriveDelay = 380;  // интервал управления мотором в мс
bool PrevForceDir = 1;
long debounceDelay = 150; // Задержка для устранения дребезга контактов в мс
long lastDebounceTime = 0; 


/* -----------------------------------------------------------
//IMU configuration
#define I2C_SDA 21
#define I2C_SCL 22
#define INTERRUPT_PIN 23  
#include "Wire.h"
#include "I2Cdev.h"

long lastImuCheckTime = 0;
long ImuCheckDelay = 200; // Интервал чтения локального IMU
#define PRINT_IMU_TO_CONSOLE // uncoment if you want to see Kayak IMU data in serial
#define PRINT_PADEL_TO_CONSOLE // uncoment if you want to see Padel IMU data in serial

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


// =========================================================================
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
// #define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

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
*/
// BLE client setup
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp32-hal-ledc.h>
static BLEUUID serviceUUID("4b2de81d-c131-4636-8d56-83b83758f7ca"); // Specify the Service UUID of Padel BLE Server
static BLEUUID CHARACTERISTIC_IMU_UUID("d2e5bfeb-d9f8-4b75-a295-d3f4032086ea"); // Добавлено
String sIMU_UUID = CHARACTERISTIC_IMU_UUID.toString().c_str(); // Добавлено
BLEClient*  pClient = nullptr;

int32_t iForceL = 0;
int32_t iForceR = 0;
  
bool doConnect = false;
bool isConnected = false;
bool doScan = false;
unsigned long lastConnectionAttempt = 0;
const unsigned long CONNECTION_ATTEMPT_INTERVAL = 5000; // 5 секунд между попытками подключения

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
    memcpy(&imuDataStruct, pData, sizeof(IMUData));
    iForceL = imuDataStruct.LoadCell_L;
    iForceR = imuDataStruct.LoadCell_R;
    #ifdef PRINT_PADEL_TO_CONSOLE
      Serial.print("Padel: "); 
      Serial.print("\t| LoadCell_L: "); Serial.print(imuDataStruct.LoadCell_L);
      Serial.print("\t| LoadCell_R: "); Serial.print(imuDataStruct.LoadCell_R);
      Serial.print("\t| ax: "); Serial.print(imuDataStruct.ax);
      Serial.print("\t ay: "); Serial.print(imuDataStruct.ay);
      Serial.print("\t az: "); Serial.print(imuDataStruct.az);
      Serial.print("\t gx: "); Serial.print(imuDataStruct.gx);
      Serial.print("\t gy: "); Serial.print(imuDataStruct.gy);
      Serial.print("\t gz: "); Serial.print(imuDataStruct.gz);
      Serial.println(); 
      // Добавьте вывод остальных данных по желанию
    #endif
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

// Scan for BLE servers and find the first one that advertises the service we are looking for. 
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks
{
 // Called for each advertising BLE server. 
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for. 
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

// button handling variable
volatile bool buttonPressed = false;

// функция вызывается по прерыванию от нажания кнопки
void buttonInterrupt() {
  buttonPressed = true;
}

// функция управления режимом работы т.е. мультипликатором через кнопку
void handleButtonPress() {
  // static unsigned long lastDebounceTime = 0;
  if ((millis() - lastDebounceTime) > debounceDelay) {
    switch (Padel_mode) {
      case 1:
        Padel_mode = 3; // Установить режим LOW
        digitalWrite(LOW_LED_PIN, HIGH);
        digitalWrite(MED_LED_PIN, LOW);
        digitalWrite(HIGH_LED_PIN, LOW);
        Serial.println("Переключение режима на LOW");
        break;
      case 2:
        Padel_mode = 1; // Установить режим POWER
        digitalWrite(LOW_LED_PIN, LOW);
        digitalWrite(MED_LED_PIN, LOW);
        digitalWrite(HIGH_LED_PIN, HIGH);
        Serial.println("Переключение режима на HIGH");
        break;
      case 3:
        Padel_mode = 2; // Установить режим MEDIUM
        digitalWrite(LOW_LED_PIN, LOW);
        digitalWrite(MED_LED_PIN, HIGH);
        digitalWrite(HIGH_LED_PIN, LOW);
        Serial.println("Переключение режима на MEDIUM");
        break;
    }
    lastDebounceTime = millis();
  }
  buttonPressed = false;
}

/*
//----------------------------------------------------
// set interrupt for MPU6050
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
//------------------------------------------------------
*/

// Функция управления мотором
void driveMotor(int32_t iForceR, int32_t iForceL) {
  int32_t absForce, adjustedForce;
  bool ForceDir;
  float modeMultiplier;
  switch (Padel_mode) {
    case 1: modeMultiplier = PWM1_HIGH; break;
    case 2: modeMultiplier = PWM1_MED; break;
    case 3: modeMultiplier = PWM1_LOW; break;
    default: modeMultiplier = PWM1_LOW;
  }

  if (std::abs(iForceR) > std::abs(iForceL)) {
    absForce = abs(iForceR);
    ForceDir = (iForceR > 0) ? 1 : 0;
  } else {
    absForce = abs(iForceL);
    ForceDir = (iForceL > 0) ? 1 : 0;
  }

  if (absForce > FORCE_THRESHOLD && ForceDir == 1) {
    adjustedForce = absForce * modeMultiplier;
    PWM1_DutyCycle = (map(adjustedForce, 0, MAX_FORCE, 1585, 2000) > 2000) ? 2000 : map(adjustedForce, 0, MAX_FORCE, 1585, 2000);
    PWM1_DutyLED = (map(adjustedForce, 0, MAX_FORCE, 0, 255) > 255) ? 255 : map(adjustedForce, 0, MAX_FORCE, 0, 255);
    servo.writeMicroseconds(PWM1_DutyCycle);
    ledcWrite(PWM1_Ch, PWM1_DutyLED);
  }
  else if (absForce > FORCE_THRESHOLD && ForceDir == 0) {
    adjustedForce = absForce * modeMultiplier;
    PWM1_DutyCycle = (map(adjustedForce, 0, MAX_FORCE, 1415, 1000) < 1000) ? 1000 : map(adjustedForce, 0, MAX_FORCE, 1415, 1000);
    PWM1_DutyLED = (map(adjustedForce, 0, MAX_FORCE, 0, 255) > 255) ? 255 : map(adjustedForce, 0, MAX_FORCE, 0, 255);
    servo.writeMicroseconds(PWM1_DutyCycle); 
    ledcWrite(PWM1_Ch, PWM1_DutyLED);
  }
  else {
    PWM1_DutyCycle = 1500;
    servo.writeMicroseconds(PWM1_DutyCycle);
    ledcWrite(PWM1_Ch, 0);
  }
  Serial.print("\t| Full Force: ");
  Serial.print(absForce);
  Serial.print("\t| adjusted to ");
  Serial.print(ForceDir ? "+ " : "- ");
  Serial.print(PWM1_DutyCycle);
  Serial.print("\t| With multiplier: ");
  Serial.println(modeMultiplier);
};

void setup() {
  Serial.begin(115200);

  // set the ADC attenuation to 11 dB (up to ~3.3V input)
  analogSetAttenuation(ADC_11db);
  
  // Initialize and arm the MOTOR
  servo.attach(MOTOR_PIN);
  servo.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
  delay(5000); // delay to allow the ESC to recognize the stopped signal.
  
 // Initialize the MOTOR_DUTY LED
  ledcAttachPin(DUTY_LED_PIN, PWM1_Ch);
  ledcWrite(PWM1_Ch, 0);
  
  // Initialize RGB indicator
  pinMode(LOW_LED_PIN, OUTPUT);
  pinMode(MED_LED_PIN, OUTPUT);
  pinMode(HIGH_LED_PIN, OUTPUT);
  digitalWrite(LOW_LED_PIN, HIGH);
  digitalWrite(MED_LED_PIN, LOW);
  digitalWrite(HIGH_LED_PIN, LOW);

  //Initialize ampifier_mode button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonInterrupt, RISING);
  Serial.println("READY, STEADY, GO!");

  //BLE setup
  BLEDevice::init("ESP32-BLE-Client on Kayak");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(1349);
  pBLEScan->setActiveScan(true);
  doScan=true;
}

void loop() {
  // BLE part
  unsigned long currentMillis = millis();

  if (!isConnected&&!doConnect) {
    if ((currentMillis - lastConnectionAttempt >= CONNECTION_ATTEMPT_INTERVAL) || doScan) {
      lastConnectionAttempt = currentMillis;
      doScan=false;
      Serial.println("Searching for a paddle...");
      BLEDevice::getScan()->start(1, scanCompletedCallback);
      // Serial.print("Stop the motor"); driveMotor(0, 0); // не ктурим мотор
    }
  }
  if (doConnect)
  {
    Serial.print("Stop the motor! "); driveMotor(0, 0); // не ктурим мотор если весло потерялось
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
    /*if ((millis() - lastImuCheckTime) > ImuCheckDelay) {
      kayak_imu(); // вычитываем данные с локального IMU
      lastImuCheckTime = millis();
    }*/
    if ((millis() - lastMotorDriveTime) > MotorDriveDelay) {
      Serial.print("Drive the motor: "); 
      driveMotor(iForceR, iForceL); // ктурим мотор
      lastMotorDriveTime = millis();
    }    
  }  

  if (buttonPressed) {
    handleButtonPress();
  }
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
/*
void kayak_imu() {
  if (!dmpReady) return;
    // read a packet from FIFO
    if (accelgyro.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      #ifdef PRINT_IMU_TO_CONSOLE
          Serial.print("Kayak: ");
      #endif
      #ifdef OUTPUT_READABLE_QUATERNION
        // display quaternion values in easy matrix form: w x y z
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        #ifdef PRINT_IMU_TO_CONSOLE
          Serial.print("\t quat: ");
          Serial.print("\t"); Serial.print(q.w);
          Serial.print("\t"); Serial.print(q.x); 
          Serial.print("\t"); Serial.print(q.y); 
          Serial.print("\t"); Serial.print(q.z); 
         #endif 
      #endif

      #ifdef OUTPUT_READABLE_EULER
        // display Euler angles in degrees
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        accelgyro.dmpGetEuler(euler, &q);
        #ifdef PRINT_IMU_TO_CONSOLE
          Serial.print("\t| euler:");
          Serial.print("\t"); Serial.print(euler[0] * 180/M_PI);
          Serial.print("\t"); Serial.print(euler[1] * 180/M_PI);
          Serial.print("\t"); Serial.print(euler[2] * 180/M_PI);
        #endif
      #endif

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        accelgyro.dmpGetGravity(&gravity, &q);
        accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #ifdef PRINT_IMU_TO_CONSOLE
          Serial.print("\t| ypr: ");
          Serial.print("\t"); Serial.print(ypr[0] * 180/M_PI); 
          Serial.print("\t"); Serial.print(ypr[1] * 180/M_PI); 
          Serial.print("\t"); Serial.print(ypr[2] * 180/M_PI);
        #endif
      #endif

      #ifdef OUTPUT_READABLE_REALACCEL
        // display real acceleration, adjusted to remove gravity
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        accelgyro.dmpGetAccel(&aa, fifoBuffer);
        accelgyro.dmpGetGravity(&gravity, &q);
        accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        #ifdef PRINT_IMU_TO_CONSOLE
          Serial.print("\t| areal: ");
          Serial.print("\t"); Serial.print(aaReal.x); 
          Serial.print("\t"); Serial.print(aaReal.y); 
          Serial.print("\t"); Serial.print(aaReal.z); 
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
          Serial.print("\t| aworld: ");
          Serial.print("\t"); Serial.print(aaWorld.x);
          Serial.print("\t"); Serial.print(aaWorld.y); 
          Serial.print("\t"); Serial.print(aaWorld.z); 
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
          Serial.print((String) "\t| BMP" + baro.type        + ": ");
          Serial.print((String) "\t P = " + baro.pressure    + " Pa."); 
          Serial.print((String) "\t T = " + baro.temperature + " *C"); 
          Serial.print((String) "\t B = " + baro.altitude    + " м."); 
      }else{
          Serial.println("Нет ответа от BPM датчика.");   
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
      Serial.print("\t| Accel: ");
      Serial.print("\t"); Serial.print(kayak_nax);
      Serial.print("\t"); Serial.print(kayak_nay);
      Serial.print("\t"); Serial.print(kayak_naz);
      
      Serial.print("\t| Gyro: ");
      Serial.print("\t"); Serial.print(kayak_ngx); 
      Serial.print("\t"); Serial.print(kayak_ngy); 
      Serial.print("\t"); Serial.print(kayak_ngz);
      
      Serial.print("\t| t*: ");
      Serial.print(kayak_ntemp);
      
      Serial.print("\t| Compass: "); 
      Serial.print("\t"); Serial.print(kayak_nmx); 
      Serial.print("\t"); Serial.print(kayak_nmy);
      Serial.print("\t"); Serial.print(kayak_nmz);
      Serial.print("\t Azimuth: "); Serial.print(kayak_azimuth);
      Serial.print("\t Bearing: "); Serial.print(kayak_bearing);
      Serial.print("\t Direction: "); Serial.print(myArray[0]); Serial.print(myArray[1]); Serial.print(myArray[2]);
     // Serial.println();

      // print the quadrobion heading, pitch and roll
      kayak_roll = filter.getRoll();
      kayak_pitch = filter.getPitch();
      kayak_heading = filter.getYaw();
      Serial.print("\t| Madgwick: "); 
      Serial.print("\t y:"); Serial.print(kayak_heading);
      Serial.print("\t p:"); Serial.print(kayak_pitch);
      Serial.print("\t r:"); Serial.print(kayak_roll);
      Serial.println();
    #endif  
}
*/