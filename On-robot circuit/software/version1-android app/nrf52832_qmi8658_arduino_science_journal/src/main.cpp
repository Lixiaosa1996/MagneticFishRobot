// TBSI-SSR LinZenan
// 2023年3月23日
// 使用arduino science journal app显示imu-qmi8658波形
#include <Arduino.h>
#include <BLEPeripheral.h>

//传感器驱动相关
#include "QMI8658C.h"
#include "I2Cdev.h"
//四元数&欧拉角计算
#include "SensorFusion.h" //SF
SF fusion;

#define DEBUG 1 // 1-串口debug
const int VERSION = 0x00000001; // 设置版本号


#define I2C_BUS          Wire                           // Define the I2C bus (Wire instance) you wish to use
I2Cdev                   i2c_0(&I2C_BUS);               // Instantiate the I2Cdev object and point to the desired I2C bus

// IIC端口号
#define SDA0 11
#define SCL0 12

// LED端口号
#define myLed_R 7

uint32_t count = 0;
uint32_t led_status = 0;

//角度到弧度转换
float pi = 3.141592653589793238462643383279502884f;


/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      * AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      * GFS_16DPS     , GFS_32DPS     , GFS_64DPS     , GFS_128DPS    , GFS_256DPS    , GFS_512DPS    , GFS_1024DPS   , GFS_2048DPS    
      * AODR_3Hz      , AODR_11Hz     , AODR_21Hz     , AODR_128Hz    , AODR_31_5Hz   , AODR_62_5Hz   , AODR_125Hz    , AODR_250Hz    , AODR_500Hz    , AODR_1000Hz   , AODR_2000Hz   , AODR_4000Hz   , AODR_8000Hz
      * GODR_31_5Hz   , GODR_62_5Hz   , GODR_125Hz    , GODR_250Hz    , GODR_500Hz    , GODR_1000Hz   , GODR_2000Hz   , GODR_4000Hz   , GODR_4000Hz, GODR_8000Hz   
*/ 
uint8_t Ascale = AFS_2G, Gscale = GFS_2048DPS, AODR = AODR_500Hz, GODR = GODR_500Hz;

float aRes, gRes;               // scale resolutions per LSB for the accel and gyro sensor2
float accelBias[3] = {0., 0., 0.}, gyroBias[3] = {0., 0., 0.}; // offset biases for the accel and gyro
int16_t QMI8658C_Data[7];        // Stores the 16-bit signed sensor output
float Gtemperature;           // Stores the real internal gyro temperature in degrees Celsius
float ax, ay, az, gx, gy, gz;  // variables to hold latest accel/gyro data values 
uint8_t QMI8658Cstatus;

bool newQMI8658C_Data = false;
bool newQMI8658CTap  = false;

QMI8658C QMI8658C(&i2c_0); // instantiate QMI8658C class

// 欧拉角计算相关参数定义
float o_gx, o_gy, o_gz, o_ax, o_ay, o_az, o_mx, o_my, o_mz;
float pitch, roll, yaw;
float cal_q[4];
float deltat;

// arduino science journal ble相关
#define SCIENCE_KIT_UUID(val) ("555a0002-" val "-467a-9538-01f0652c74e8")

// custom boards may override default pin definitions with BLEPeripheral(PIN_REQ, PIN_RDY, PIN_RST)
BLEPeripheral blePeripheral = BLEPeripheral();

BLEService service(SCIENCE_KIT_UUID("0000"));
BLEUnsignedIntCharacteristic versionCharacteristic(SCIENCE_KIT_UUID("0001"), BLERead);
BLECharacteristic accelerationCharacteristic(SCIENCE_KIT_UUID("0011"), BLENotify, 3 * sizeof(float));
BLECharacteristic gyroscopeCharacteristic(SCIENCE_KIT_UUID("0012"), BLENotify, 3 * sizeof(float));
BLECharacteristic tempCharacteristic(SCIENCE_KIT_UUID("0014"), BLENotify, 1 * sizeof(float));


boolean significantChange(float val1, float val2, float threshold) {
  return (abs(val1 - val2) >= threshold);
}

void blePeripheralConnectHandler(BLECentral& central) {
  Serial.print(F("Connected event, central: "));
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  Serial.print(F("Disconnected event, central: "));
  Serial.println(central.address());
}

// 读取IMU数据并发送到ble service
void setIMUCharacteristicValue() {
// 读取imu原始数据
  QMI8658C.readData(QMI8658C_Data); 
   
  // // Now we'll calculate the accleration value into actual g's
  // ax = (float)QMI8658C_Data[1]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
  // ay = (float)QMI8658C_Data[2]*aRes - accelBias[1];   
  // az = (float)QMI8658C_Data[3]*aRes - accelBias[2];  
  // Serial.print("aRes*10000 = ");
  // Serial.println(aRes*10000);
  // Serial.print("gRes*10000 = ");
  // Serial.println(gRes*10000);

  ax = (float)QMI8658C_Data[1]*aRes;  //  - accelBias[0] get actual g value, this depends on scale being set
  ay = (float)QMI8658C_Data[2]*aRes;  //  - accelBias[1]
  az = (float)QMI8658C_Data[3]*aRes;  //  - accelBias[2]
  // ax = (float)QMI8658C_Data[1]*aRes - accelBias[0];  //   get actual g value, this depends on scale being set
  // ay = (float)QMI8658C_Data[2]*aRes - accelBias[1];  //
  // az = (float)QMI8658C_Data[3]*aRes - accelBias[2];  // 

  // 去除零偏
  gx = (float)QMI8658C_Data[4]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
  gy = (float)QMI8658C_Data[5]*gRes - gyroBias[1];  
  gz = (float)QMI8658C_Data[6]*gRes - gyroBias[2];
  
  // 温度转化 
  Gtemperature = (float) QMI8658C_Data[0]/256.0f; // Gyro chip temperature in degrees Centigrade
  
  // 陀螺仪 转为弧度制
  gx = gx/180.0*pi;  //  get actual gyro value, this depends on scale being set
  gy = gy/180.0*pi;  // 
  gz = gz/180.0*pi;  // 

  // 写入acc characteristic
  float acceleration[3];
  acceleration[0] = ax;
  acceleration[1] = ay;
  acceleration[2] = az;
  accelerationCharacteristic.setValue((byte*)acceleration, sizeof(acceleration));

  // 写入acc characteristic
  float gyroscope[3];
  gyroscope[0] = gx;
  gyroscope[1] = gy;
  gyroscope[2] = gz;
  gyroscopeCharacteristic.setValue((byte*)gyroscope, sizeof(gyroscope));

  // 写入temp characteristic
  tempCharacteristic.setValue((uint8_t*)&Gtemperature, sizeof(float));

  // 打印imu数据
  #ifdef DEBUG
    Serial.print("ACCX = ");
    Serial.print(acceleration[0]);
    Serial.print("; ACCY = ");
    Serial.print(acceleration[1]);
    Serial.print("; ACCZ = ");
    Serial.println(acceleration[2]);
    
    Serial.print("GYROX = ");
    Serial.print(gyroscope[0]);
    Serial.print("; GYROY = ");
    Serial.print(gyroscope[1]);
    Serial.print("; GYROZ = ");
    Serial.println(gyroscope[2]);

    Serial.print("TEMP = ");
    Serial.println(Gtemperature);
  #endif
}


void setup() {
  // uart init
  Serial.begin(115200);

  // Configure led
  pinMode(myLed_R, OUTPUT);
  digitalWrite(myLed_R, LOW); // start with led on

  // IIC init
  Wire.begin(); // set master mode 
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(1000);
  
  // i2c_0.I2Cscan();
  digitalWrite(myLed_R, HIGH);
  delay(1000);
  digitalWrite(myLed_R, LOW);
  delay(1000);
  digitalWrite(myLed_R, HIGH);
  delay(1000);
  digitalWrite(myLed_R, LOW);
  delay(1000);

  // Read the QMI8658C Chip ID register, this is a good test of communication
  Serial.println("QMI8658C accel/gyro...");
  byte c = QMI8658C.getChipID();  // Read CHIP_ID register for QMI8658C
  Serial.print("QMI8658C "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x05, HEX);
  Serial.println(" ");
  delay(1000); 
  if(c == 0x05) // check if all I2C sensors have acknowledged
  {
      Serial.println("QMI8658C are online..."); Serial.println(" ");
      
      digitalWrite(myLed_R, HIGH);
      delay(1000);
      digitalWrite(myLed_R, LOW);
  
      // get sensor resolutions, only need to do this once
      aRes = QMI8658C.getAres(Ascale);
      gRes = QMI8658C.getGres(Gscale);

      Serial.print("aRes*10000 = ");
      Serial.println(aRes*10000);
      Serial.print("gRes*10000 = ");
      Serial.println(gRes*10000);
  
      QMI8658C.init(Ascale, Gscale, AODR, GODR);
  
      QMI8658C.offsetBias(accelBias, gyroBias);
      Serial.println("accel biases (mg)"); Serial.println(1000.0f * accelBias[0]); Serial.println(1000.0f * accelBias[1]); Serial.println(1000.0f * accelBias[2]);
      Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
      delay(1000); 
  
      // digitalWrite(myLed_R, HIGH);
  }
  else 
  {
    if(c != 0x05) Serial.println(" QMI8658C not functioning!");
    while(1){}; //不成功就阻塞
  }

  blePeripheral.setLocalName("SSR-Zenan-NRF-IMU");

  blePeripheral.setAdvertisedServiceUuid(service.uuid());
  blePeripheral.addAttribute(service);
  blePeripheral.addAttribute(versionCharacteristic);
  blePeripheral.addAttribute(accelerationCharacteristic);
  blePeripheral.addAttribute(gyroscopeCharacteristic);
  blePeripheral.addAttribute(tempCharacteristic);
  // service.addCharacteristic(versionCharacteristic);
  // service.addCharacteristic(accelerationCharacteristic);
  // service.addCharacteristic(gyroscopeCharacteristic);

  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  blePeripheral.begin();

  Serial.println(F("BLE IMU Sensor Peripheral"));
  versionCharacteristic.setValue(VERSION);
}

void loop() {
  blePeripheral.poll();
  setIMUCharacteristicValue();
  delay(100); // 10hz
  count++; // 统计粗略的1s
  if(count%10==1)
  {
    digitalWrite(myLed_R, led_status);
    led_status = 1-led_status;
  }

  // digitalWrite(myLed_R, HIGH);
  // delay(1000);
  // digitalWrite(myLed_R, LOW);
  // delay(1000);
  // Serial.println("hello...");
}

