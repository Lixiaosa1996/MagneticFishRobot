#include <Arduino.h>
// add 定时器 library
#include "Ticker.h"

// add 传感器驱动相关 library
#include "QMI8658C.h"
#include "I2Cdev.h"
#define I2C_BUS          Wire                           // Define the I2C bus (Wire instance) you wish to use
I2Cdev                   i2c_0(&I2C_BUS);               // Instantiate the I2Cdev object and point to the desired I2C bus
// IIC端口号
#define SDA0 11
#define SCL0 12
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

// add BLE server library
#include <BLEPeripheral.h>

// service & characteristic uuid 设置
#define SERVICE_UUID        "8a90e102-edde-11e6-bc64-92361f002671"      
#define ADC_CHARACTERISTIC_UUID "8a90acc1-edde-11e6-bc64-92361f002671"
#define QUAT_CHARACTERISTIC_UUID "8a90acc2-edde-11e6-bc64-92361f002671"

// BLE配置相关 (发布一个characteristic代表adc的short数据)
BLEPeripheral blePeripheral = BLEPeripheral();
BLEService service(SERVICE_UUID);
BLECharacteristic adcCharacteristic((ADC_CHARACTERISTIC_UUID), BLENotify, 1 * sizeof(short));
BLECharacteristic quatCharacteristic((QUAT_CHARACTERISTIC_UUID), BLENotify, 4 * sizeof(float));

void blePeripheralConnectHandler(BLECentral& central) {
  Serial.print(F("Connected event, central: "));
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  Serial.print(F("Disconnected event, central: "));
  Serial.println(central.address());
}

// ADC 端口号
// #define AD0 16  // ADC1
#define AD0 19  // BAT_ADC
short adc_Value = 0;

// LED 工作指示相关
#define LED0 7 //8 // IO19
bool led_status = false;
// LED 闪烁任务回调函数
void blinky(void)
{
  // Toggle LED0
  digitalWrite(LED0, led_status);
  led_status = !led_status;
}
// 新建 LED 闪烁任务 1s 闪烁
Ticker blinkLED (blinky, 1000); 

// add 四元数计算 library
#include "SensorFusion.h" //SF
SF fusion;
// 四元数计算相关参数定义
float o_gx, o_gy, o_gz, o_ax, o_ay, o_az, o_mx, o_my, o_mz;
float pitch, roll, yaw;
float cal_q[4];
float deltat;
//角度到弧度转换
float pi = 3.141592653589793238462643383279502884f;
// 四元数 计算任务回调函数
void cal_quat(void)
{ 
  // 01-采集数据
  // 读取imu原始数据
  QMI8658C.readData(QMI8658C_Data); 
  ax = (float)QMI8658C_Data[1] * aRes; 
  ay = (float)QMI8658C_Data[2] * aRes;
  az = (float)QMI8658C_Data[3] * aRes;
  gx = (float)QMI8658C_Data[4] * gRes - gyroBias[0]; 
  gy = (float)QMI8658C_Data[5] * gRes - gyroBias[1]; 
  gz = (float)QMI8658C_Data[6] * gRes - gyroBias[2];
  // 加速度计 转为m/s2
  ax = ax * 9.8f;
  ay = ay * 9.8f;
  az = az * 9.8f;
  // 陀螺仪 转为rad/s
  gx = gx / 180.0 * pi;  
  gy = gy / 180.0 * pi;
  gz = gz / 180.0 * pi;

  // 02-更新四元数
  deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
  fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  pitch = fusion.getPitch();
  roll = fusion.getRoll();
  yaw = fusion.getYaw();
  cal_q[0] = fusion.getQuat()[0];
  cal_q[1] = fusion.getQuat()[1];
  cal_q[2] = fusion.getQuat()[2];
  cal_q[3] = fusion.getQuat()[3];
}
// 新建  四元数 计算任务 100ms
Ticker calQUAT (cal_quat, 100);


void ble_send_data() {
    // 01-写入 ADC characteristic
    adc_Value = analogRead(AD0);
    adcCharacteristic.setValue((uint8_t*)&adc_Value, sizeof(short));
    Serial.print("ADC val: ");
    Serial.println(adc_Value);  

    // 02-写入 QUAT characteristic
    quatCharacteristic.setValue((uint8_t*)&cal_q, sizeof(cal_q));
    Serial.print("QUAT val: ");
    Serial.print(cal_q[0]);
    Serial.print(" ");
    Serial.print(cal_q[1]);
    Serial.print(" ");
    Serial.print(cal_q[2]);
    Serial.print(" ");
    Serial.println(cal_q[3]);
    Serial.println(" ");
  }

// BLE 发送任务相关
// BLE 发送任务回调函数
void blesend(void)
{
  // SEND DATA
  ble_send_data();
}
// 新建 BLE 发送任务 0.5s 间隔
Ticker bleSEND (blesend, 500); 



void setup() {
    // 00-Configure uart output
    Serial.begin(115200);

    // 01-Configure adc & led
    pinMode(AD0, INPUT); 
    pinMode(LED0, OUTPUT); 
    delay(500);

    // 02-IMU configure
    Wire.begin(); // set master mode 
    Wire.setClock(400000); // I2C frequency at 400 kHz  
    delay(500);
    i2c_0.I2Cscan();
    digitalWrite(LED0, HIGH);
    delay(500);
    digitalWrite(LED0, LOW);
    delay(500);
    digitalWrite(LED0, HIGH);
    delay(500);
    digitalWrite(LED0, LOW);
    delay(500);
    // Read the QMI8658C Chip ID register, this is a good test of communication
    Serial.println("QMI8658C accel/gyro...");
    byte c = QMI8658C.getChipID();  // Read CHIP_ID register for QMI8658C
    Serial.print("QMI8658C "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x05, HEX);
    Serial.println(" ");
    delay(500); 
    if(c == 0x05) // check if all I2C sensors have acknowledged
    {
        Serial.println("QMI8658C are online..."); Serial.println(" ");
        
        digitalWrite(LED0, HIGH);
        delay(500);
        digitalWrite(LED0, LOW);
    
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
        }
    else 
    {
      if(c != 0x05) Serial.println(" QMI8658C not functioning!");
      while(1){}; //不成功就阻塞
    }
   
    // 03-BLE configure
    blePeripheral.setLocalName("NRF-IMU-SSR-zenan");
    blePeripheral.setAdvertisedServiceUuid(service.uuid());
    blePeripheral.addAttribute(service);
    blePeripheral.addAttribute(adcCharacteristic); // adc 特征
    blePeripheral.addAttribute(quatCharacteristic); //  四元数 特征
    blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
    blePeripheral.begin();
    Serial.println(F("BLE ADC Sensor Peripheral"));

    // 04-Start the Timer function to avoid using delay()
    blinkLED.start();
    bleSEND.start();
    calQUAT.start();
}



void loop() {
    // 开启ble功能
    blePeripheral.poll();

    // Keep feeding timers
    blinkLED.update();
    bleSEND.update();
    calQUAT.update();
}

