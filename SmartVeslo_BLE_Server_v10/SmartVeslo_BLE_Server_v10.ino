// Smart Veslo by Baga
//Padel with two tenzosensors and BLE Server

// power control pin power is on during HIGH
#define POWER_PIN 1 // power is ON during HIGH
// ESP32C3v2 Supermini RGB onboard LED
#define RGB_BRIGHTNESS 64 // Change white brightness (max 255)
#define RGB_BUILTIN 8 // PIN for RGB onboard LED

// HX711 tenzosensor configuration
#include "HX711.h"
  constexpr float CALIBRATION_FACTOR_L = 27.63158;
  constexpr float CALIBRATION_FACTOR_R = -24.07894;
  constexpr int RIGHT_LOADCELL_DOUT_PIN = 2;
  constexpr int RIGHT_LOADCELL_SCK_PIN = 3;
  constexpr int LEFT_LOADCELL_DOUT_PIN = 5;
  constexpr int LEFT_LOADCELL_SCK_PIN = 6; 
HX711 scaleR;
HX711 scaleL;

// Barometer configuration
#include <iarduino_Pressure_BMP.h> 
iarduino_Pressure_BMP baro; 


// BLE server configuration
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#define SERVICE_UUID        "4b2de81d-c131-4636-8d56-83b83758f7ca"
//#define CHARACTERISTIC_FORCE_L_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
//#define CHARACTERISTIC_FORCE_R_UUID "cba1d466-344c-4be3-ab3f-189f80dd7518"
#define CHARACTERISTIC_IMU_UUID "d2e5bfeb-d9f8-4b75-a295-d3f4032086ea"
//BLECharacteristic *forceL;
//BLECharacteristic *forceR;
BLECharacteristic *imuData;
// BLE server variables
BLEServer *pServer = NULL;
BLEService *pService = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// класс для подключения и отключения устройства
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("Connected");
      neopixelWrite(RGB_BUILTIN,0,RGB_BRIGHTNESS,0); // Green  
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      Serial.println("Disconnected");
      deviceConnected = false;
      neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,0); // Red
    }
};

//IMU configuration
#define I2C_SDA 9
#define I2C_SCL 10
#define INTERRUPT_PIN 0  
#include "Wire.h"
#include "I2Cdev.h"
//#include "MPU6050.h" // подключаем вместо следующей
#include "MPU6050_6Axis_MotionApps20.h"
#include <QMC5883LCompass.h> // ebuchi compas желательно поменять на что-то работающее

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


// set interrupt for MPU6050
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

MPU6050 accelgyro;
int16_t ax, ay, az;
float nax, nay, naz;
int16_t gx, gy, gz;
float ngx, ngy, ngz;
int16_t temperature;
float ntemp;

QMC5883LCompass compass;
int mx, my, mz, azimuth, bearing;
float nmx, nmy, nmz;

// Mdgwick lib
#include <MadgwickAHRS.h>
Madgwick filter;
float roll, pitch, heading;


void setup() {
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);
  Serial.begin(115200);
  Serial.println("Smart Padel by Baga");

  // LoadCell Initialization and Calibration 
  Serial.println("Initializing HX711 LoadCells");
  scaleR.begin(RIGHT_LOADCELL_DOUT_PIN, RIGHT_LOADCELL_SCK_PIN);
  scaleL.begin(LEFT_LOADCELL_DOUT_PIN, LEFT_LOADCELL_SCK_PIN);
  LoadCellCalibrateSensors();
 
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
    accelgyro.setXAccelOffset(-1003);
    accelgyro.setYAccelOffset(-465);
    accelgyro.setZAccelOffset(1353);
    accelgyro.setXGyroOffset(90);
    accelgyro.setYGyroOffset(33);
    accelgyro.setZGyroOffset(37);   
  
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
/*
  Wire.beginTransmission(0x68);
  Wire.write(0x6A);
  Wire.write(0x00);
  Wire.endTransmission();

  //Disable Sleep Mode
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();  
*/
  
// Madgwick initialization
Serial.println("Initializing Madgwick Quadrobion");
  filter.begin(25);


// BLE Server initialization
  Serial.println("Creating BLE Server and Characteristic");
  BLEDevice::init("SmartVeslo");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  pService = pServer->createService(SERVICE_UUID);
  // Создание характеристики для IMU
  imuData = pService->createCharacteristic(
    CHARACTERISTIC_IMU_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  imuData->setValue("0");
  /*
  forceL = pService->createCharacteristic(
    CHARACTERISTIC_FORCE_L_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  forceL->setValue("0");

  forceR = pService->createCharacteristic(
    CHARACTERISTIC_FORCE_R_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  forceR->setValue("0");
  */
  Serial.println("Starting BLE Server");
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  neopixelWrite(RGB_BUILTIN,0,0,RGB_BRIGHTNESS); // Blue
  Serial.println("Waiting for BLE connection...");
  }
 
void loop() {
   digitalWrite(POWER_PIN, HIGH); // почему-то в setup оказалось не достаточно
   if (!deviceConnected && oldDeviceConnected) {
    delay(500); // delay for BLE stack
    pServer->startAdvertising(); // restart advertising
    Serial.println("BLE advertising started");
    oldDeviceConnected = deviceConnected;
  }
  // connection established
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
  
  if (deviceConnected) {
    // HX711 tenzosensors reading
    int32_t LoadCell_R = (int32_t)scaleR.get_units(1)*-1;
    int32_t LoadCell_L = (int32_t)scaleL.get_units(1)*-1;
    scaleR.power_down();			        // put the ADC in sleep mode
    scaleL.power_down();			        // put the ADC in sleep mode
    if (LoadCell_R == INT32_MAX || LoadCell_L == INT32_MAX) {
      Serial.println("Warning: possible data overflow!");
    }   

    if (!dmpReady) return;
    // read a packet from FIFO
    if (accelgyro.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      #ifdef OUTPUT_READABLE_QUATERNION
        // display quaternion values in easy matrix form: w x y z
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        Serial.print("quat\t");
        Serial.print(q.w);
        Serial.print("\t");
        Serial.print(q.x);
        Serial.print("\t");
        Serial.print(q.y);
        Serial.print("\t");
        Serial.println(q.z);
      #endif

      #ifdef OUTPUT_READABLE_EULER
        // display Euler angles in degrees
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        accelgyro.dmpGetEuler(euler, &q);
        Serial.print("euler\t");
        Serial.print(euler[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(euler[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(euler[2] * 180/M_PI);
      #endif

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        accelgyro.dmpGetGravity(&gravity, &q);
        accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
      #endif

      #ifdef OUTPUT_READABLE_REALACCEL
        // display real acceleration, adjusted to remove gravity
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        accelgyro.dmpGetAccel(&aa, fifoBuffer);
        accelgyro.dmpGetGravity(&gravity, &q);
        accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        Serial.print("areal\t");
        Serial.print(aaReal.x);
        Serial.print("\t");
        Serial.print(aaReal.y);
        Serial.print("\t");
        Serial.println(aaReal.z);
      #endif

      #ifdef OUTPUT_READABLE_WORLDACCEL
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        accelgyro.dmpGetAccel(&aa, fifoBuffer);
        accelgyro.dmpGetGravity(&gravity, &q);
        accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        accelgyro.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        Serial.print("aworld\t");
        Serial.print(aaWorld.x);
        Serial.print("\t");
        Serial.print(aaWorld.y);
        Serial.print("\t");
        Serial.println(aaWorld.z);
      #endif    
    }  

    // IMU Reading RAW data 
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); 
    temperature = accelgyro.getTemperature(); 

    compass.read(); //сраный компас так нормально и не заработал
      mx = compass.getX();
      my = compass.getY();
      mz = compass.getZ();
      azimuth = compass.getAzimuth();
      bearing = compass.getBearing(azimuth);
      char myArray[3];
      compass.getDirection(myArray, azimuth);

    //barometer read - не знаю зачем он нам но раз есть я его подключил
    if( baro.read(2) ){ // Функция read(2) читает давление в Па, температуру в °С и высоту в м.
         Serial.print((String) "Датчик BMP"+ baro.type        + ": ");
         Serial.print((String) "P = "      + baro.pressure    + " Pa, \t");
         Serial.print((String) "T = "      + baro.temperature + " *C, \t\t");
         Serial.print((String) "B = "      + baro.altitude    + " м.\r\n");
     }else{
         Serial.println("Нет ответа от датчика.");   
     }

    // convert from raw data to gravity and degrees/second units
    nax = convertRawAcceleration(ax);
    nay = convertRawAcceleration(ay);
    naz = convertRawAcceleration(az);
    ngx = convertRawGyro(gx);
    ngy = convertRawGyro(gy);
    ngz = convertRawGyro(gz);
    ntemp = convertTemp(temperature);
    nmx = convertRawCompas(mx);
    nmy = convertRawCompas(my);
    nmz = convertRawCompas(mz);


    // update the квадробион, which computes orientation
    filter.update(ngx, ngy, ngz, nax, nay, naz, nmx, nmy, nmz);

    // выводим показатели в серийный порт
    // print LoadCell values to console
    Serial.print("LEFT: ");
    Serial.print((int)LoadCell_L, DEC);
    Serial.print("\t| RIGHT: ");
    Serial.print((int)LoadCell_R, DEC);
  
    // print IMU values
    Serial.print("\t| acc ");
    Serial.print(nax); Serial.print(" ");
    Serial.print(nay); Serial.print(" ");
    Serial.print(naz);
    
    Serial.print("\t| gyr ");
    Serial.print(ngx); Serial.print(" ");
    Serial.print(ngy); Serial.print(" ");
    Serial.print(ngz);
    
    Serial.print("| t* ");
    Serial.print(ntemp);
    
    Serial.print(" | mag"); 
    Serial.print(nmx); Serial.print(" ");
    Serial.print(nmy); Serial.print(" ");
	  Serial.print(nmz); Serial.print(" ");
	  Serial.print(" Azimuth: ");	Serial.print(azimuth);
	  Serial.print(" Bearing: ");	Serial.print(bearing);
    Serial.print(" Direction: "); Serial.print(myArray[0]); Serial.print(myArray[1]); Serial.print(myArray[2]);
    Serial.println();

    // print the quadrobion heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("Madgwick Q*: "); 
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);

  // Создание структуры для хранения данных IMU
  struct IMUData {
    int32_t LoadCell_L, LoadCell_R;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int mx, my, mz;
    int azimuth;
    float roll, pitch, heading;
  } imuDataStruct;

  // Заполнение структуры данными
  imuDataStruct = {LoadCell_L, LoadCell_R, ax, ay, az, gx, gy, gz, mx, my, mz, azimuth, roll, pitch, heading};

  // Отправка данных IMU через BLE
  // push IMU data to BLE Client
  imuData->setValue((uint8_t*)&imuDataStruct, sizeof(IMUData));
  imuData->notify();

  /*
    // push tenzodata to BLE Client
    forceL->setValue((uint8_t*)&LoadCell_L,sizeof(int32_t));
    forceR->setValue((uint8_t*)&LoadCell_R,sizeof(int32_t));;
    forceL->notify();
    forceR->notify();
  */
  
  //delay(300);
  
  scaleR.power_up();             // put the ADC in power up mode
  scaleL.power_up();             // put the ADC in power up mode

  } else {
  //  esp_sleep_enable_timer_wakeup(500); 
  //  esp_light_sleep_start();
  };  
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

// Calibration factor calculation function for LoadCell
void LoadCellcalibrationFactor() {
 if (scaleR.is_ready()) {
    scaleR.set_scale();    
    Serial.println("Tare... remove any dutys from the RIGHT.");
    delay(5000);
    Serial.println("Measuring...");
    scaleR.tare();
    Serial.println("Tare done...");
    Serial.print("Place a known duty on the RIGHT...");
    delay(5000);
    Serial.println("Measuring...");
    long reading = scaleR.get_units(10);
    Serial.print("Result: ");
    Serial.println(reading);
    Serial.println("calibration factor will be the (Result)/(known duty)");
  }
  else {
    Serial.println("RIGHT HX711 not found.");
  };

  if (scaleL.is_ready()) {
    scaleL.set_scale();    
    Serial.println("Tare... remove any dutys from the LEFT.");
    delay(5000);
    Serial.println("Measuring...");
    scaleL.tare();
    Serial.println("Tare done...");
    Serial.print("Place a known duty on the LEFT...");
    delay(5000);
    Serial.println("Measuring...");
    long reading = scaleL.get_units(10);
    Serial.print("Result: ");
    Serial.println(reading);
    Serial.println("calibration factor will be the (Result)/(known duty)");
  }
  else {
    Serial.println("LEFT HX711 not found.");
  };
  delay(1000);
//calibration factor will be the (reading)/(known duty)
}


//Tenzosensors calibration function
void LoadCellCalibrateSensors() {

  Serial.println("Before setting up the RIGHT side scale:");
  Serial.print("read average: \t");
  Serial.println(scaleR.read_average(20));  	// print the average of 20 readings from the ADC
  Serial.print("get value: \t");
  Serial.println(scaleR.get_value(5));		// print the average of 5 readings from the ADC minus the tare duty (not set yet)
  Serial.print("get units: \t");
  Serial.println(scaleR.get_units(5), 1);	// print the average of 5 readings from the ADC minus tare duty (not set) divided by the SCALE parameter (not set yet)

  scaleR.set_scale(CALIBRATION_FACTOR_R);     // this value is obtained by calibrating the scale with known dutys; see the README for details
  scaleR.tare();   // reset the scale to 0
  
  Serial.println("After setting up the RIGHT side scale:");
  Serial.print("read average: \t");
  Serial.println(scaleR.read_average(20));       // print the average of 20 readings from the ADC
  Serial.print("get value: \t");
  Serial.println(scaleR.get_value(5));		// print the average of 5 readings from the ADC minus the tare duty, set with tare()
  Serial.print("get units: \t");
  Serial.println(scaleR.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare duty, divided by the SCALE parameter set with set_scale

  Serial.println("Before setting up the LEFT side scale:");
  Serial.print("read average: \t");
  Serial.println(scaleL.read_average(20));  	// print the average of 20 readings from the ADC
  Serial.print("get value: \t");
  Serial.println(scaleL.get_value(5));		// print the average of 5 readings from the ADC minus the tare duty (not set yet)
  Serial.print("get units: \t");
  Serial.println(scaleL.get_units(5), 1);	// print the average of 5 readings from the ADC minus tare duty (not set) divided by the SCALE parameter (not set yet)

  scaleL.set_scale(CALIBRATION_FACTOR_L);     // this value is obtained by calibrating the scale with known dutys; see the README for details
  scaleL.tare();   // reset the scale to 0
  
  Serial.println("After setting up the LEFT side scale:");
  Serial.print("read average: \t");
  Serial.println(scaleL.read_average(20));       // print the average of 20 readings from the ADC
  Serial.print("get value: \t");
  Serial.println(scaleL.get_value(5));		// print the average of 5 readings from the ADC minus the tare duty, set with tare()
  Serial.print("get units: \t");
  Serial.println(scaleL.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare duty, divided by the SCALE parameter set with set_scale
}

