#include <Arduino.h>
#include "BLEDevice.h"

// #define debug

//cup为小端模式存储，也就是在存储的时候，低位被存在0字节，高位在1字节
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))     //取出int型变量的低字节
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))     //    取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

// some parameters 
unsigned char DataToSend[30];//send to ANOV7

//BLE Server name (BLE发送者名字)
#define bleServerName "NRF-IMU-SSR-zenan"
// service & characteristic uuid 设置
#define SERVICE_UUID        "8a90e102-edde-11e6-bc64-92361f002671"      
#define ADC_CHARACTERISTIC_UUID "8a90acc1-edde-11e6-bc64-92361f002671"
#define QUAT_CHARACTERISTIC_UUID "8a90acc2-edde-11e6-bc64-92361f002671"

/* UUID's of the service, characteristic that we want to read*/
// BLE Services
static BLEUUID ServiceUUID(SERVICE_UUID);

// Characteristic
static BLEUUID adcCharacteristicUUID(ADC_CHARACTERISTIC_UUID);
static BLEUUID quatCharacteristicUUID(QUAT_CHARACTERISTIC_UUID);

//Flags stating if should begin connecting and if the connection is up
static boolean doConnect = false;
static boolean connected = false;

//Address of the peripheral device. Address will be found during scanning...
static BLEAddress *pServerAddress;
 
//Characteristicd that we want to read
static BLERemoteCharacteristic* adcCharacteristic;
static BLERemoteCharacteristic* quatCharacteristic;


//Activate notify
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};

//Variables to store temperature and quat
char* adcChar;
short adc;
char* quatChar;
float quat[4];
int16_t quatx10000[4];

//Flags to check whether new adc readings are available
boolean newadc = false;
boolean newquat = false;

//When the BLE Server sends a new adc reading with the notify property
static void adcNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                        uint8_t* pData, size_t length, bool isNotify) {
  //store adc value
  adcChar = (char*)pData;
  memcpy (&adc, pData, 2);
  newadc = true;
}

//When the BLE Server sends a new quat reading with the notify property
static void quatNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                        uint8_t* pData, size_t length, bool isNotify) {
  //store quat value
  quatChar = (char*)pData;
  memcpy (&quat, pData, 16);
  newquat = true;
}

//Connect to the BLE Server that has the name, Service, and Characteristics
bool connectToServer(BLEAddress pAddress) {
   BLEClient* pClient = BLEDevice::createClient();
 
  // Connect to the remove BLE Server.
  pClient->connect(pAddress, BLE_ADDR_TYPE_RANDOM);  // must be random type when connect to a nrf server
  Serial.println(" - Connected to server");
 
  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(ServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(ServiceUUID.toString().c_str());
    return (false);
  }
 
  // Obtain a reference to the characteristics in the service of the remote BLE server.
  adcCharacteristic = pRemoteService->getCharacteristic(adcCharacteristicUUID);
  quatCharacteristic = pRemoteService->getCharacteristic(quatCharacteristicUUID);

  if (adcCharacteristic == nullptr || quatCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID");
    return false;
  }

  Serial.println(" - Found our characteristics");
 
  //Assign callback functions for the Characteristics
  adcCharacteristic->registerForNotify(adcNotifyCallback);
  quatCharacteristic->registerForNotify(quatNotifyCallback);

  return true;
}

//Callback function that gets called, when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == bleServerName) { //Check if the name of the advertiser matches
      advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser is the one we need
      doConnect = true; //Set indicator, stating that we are ready to connect
      Serial.println("Device found. Connecting!");
    }
  }
};
 


//function that prints the latest sensor readings in the OLED display
void printReadings(){
  Serial.print("ADC val:");
  Serial.println(adc);

  Serial.print("QUAT val: ");
  Serial.print(quat[0]);
  Serial.print(" ");
  Serial.print(quat[1]);
  Serial.print(" ");
  Serial.print(quat[2]);
  Serial.print(" ");
  Serial.println(quat[3]);
  Serial.println(" ");
}


//ANOv7上位机波形观察 四元数
void ANOv7_QUAT_DATA_Send(void)
{
  unsigned char _cnt_send = 0;//用于记录发送帧发送位数
  unsigned char SC = 0;//求和检验值
  unsigned char AC = 0;//累求和检验值
  unsigned long time; 

  //帧头
  DataToSend[_cnt_send++] = 0xAA;
  DataToSend[_cnt_send++] = 0xFF;
  DataToSend[_cnt_send++] = 0x04;
  DataToSend[_cnt_send++] = 4*2+1;  //short发送占两个字节

  // 四元数回传
  quatx10000[0] = int16_t(10000.0f * quat[0]);
  quatx10000[1] = int16_t(10000.0f * quat[1]);
  quatx10000[2] = int16_t(10000.0f * quat[2]);
  quatx10000[3] = int16_t(10000.0f * quat[3]);

  DataToSend[_cnt_send++] = BYTE0(quatx10000[0]);
  DataToSend[_cnt_send++] = BYTE1(quatx10000[0]);
  DataToSend[_cnt_send++] = BYTE0(quatx10000[1]);
  DataToSend[_cnt_send++] = BYTE1(quatx10000[1]);
  DataToSend[_cnt_send++] = BYTE0(quatx10000[2]);
  DataToSend[_cnt_send++] = BYTE1(quatx10000[2]);
  DataToSend[_cnt_send++] = BYTE0(quatx10000[3]);
  DataToSend[_cnt_send++] = BYTE1(quatx10000[3]);

  DataToSend[_cnt_send++] = 0; //fusion_sta

  //双校验
  for(unsigned char i = 0;i < (4*2+1+4);i++)
  {
    SC = DataToSend[i] + SC;
    AC = SC + AC;
  }

  DataToSend[_cnt_send++] = SC;
  DataToSend[_cnt_send++] = AC;

  //发送
  for(unsigned char i = 0;i < _cnt_send;i++)
  {
      Serial.write(DataToSend[i]);
  }
}

void setup() {
  //Start serial communication
  Serial.begin(115200);
  Serial.println("Starting BLE Client application...");

  //Init BLE device
  BLEDevice::init("");
 
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(20);
}

void loop() {
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
      //Activate the Notify property of each Characteristic(激活订阅)
      adcCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
      quatCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);

      connected = true;
    } else {
      Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
    }
    doConnect = false;
  }

    if (newadc && newquat){
    newadc = false;
    newquat = false;
    #ifdef debug
      printReadings();
    #else
      ANOv7_QUAT_DATA_Send();
    #endif
  }
  delay(1000); // Delay a second between loops.
}