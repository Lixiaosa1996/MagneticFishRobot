/* 05/03/2022
    Created by ZenanLin
    Library may be used freely and without limit with attribution.
*/

#include "QMI8658C.h"
#include "I2Cdev.h"

QMI8658C::QMI8658C(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}


uint8_t QMI8658C::getChipID()
{
  uint8_t c = _i2c_bus->readByte(QMI8658C_ADDRESS, QMI8658C_WHO_AM_I);
  return c;
}


float QMI8658C::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
      _aRes = 2.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_4G:
      _aRes = 4.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_8G:
      _aRes = 8.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_16G:
      _aRes = 16.0f / 32768.0f;
      return _aRes;
      break;
  }
}

float QMI8658C::getGres(uint8_t Gscale) {
  switch (Gscale)
  {
    case GFS_16DPS     :
      _gRes = 16.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_32DPS     :
      _gRes = 32.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_64DPS:
      _gRes = 64.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_128DPS:
      _gRes = 128.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_256DPS:
      _gRes = 256.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_512DPS:
      _gRes = 512.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_1024DPS:
      _gRes = 1024.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_2048DPS:
      _gRes = 2048.0f / 32768.0f;
      return _gRes;
      break;
  }
}


void QMI8658C::reset()
{
  // reset device
  uint8_t temp = _i2c_bus->readByte(QMI8658C_ADDRESS, QMI8658C_DEVICE_CONFIG);
  _i2c_bus->writeByte(QMI8658C_ADDRESS, QMI8658C_DEVICE_CONFIG, temp | 0x01); // Set bit 0 to 1 to reset QMI8658C
  delay(1); // Wait for all registers to reset
}


uint8_t QMI8658C::status()
{
  // reset device
  uint8_t temp = _i2c_bus->readByte(QMI8658C_ADDRESS, QMI8658C_INT_STATUS);
  return temp;
}


// 对器件进行9轴量程和采样速率做更新
void QMI8658C::init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR)
{
  // first config
  uint8_t temp = _i2c_bus->readByte(QMI8658C_ADDRESS, QMI8658C_CTRL1); // make sure not to disturb reserved bit values
  _i2c_bus->writeByte(QMI8658C_ADDRESS, QMI8658C_CTRL1, 0x40);  //  acc full scale and data rate
  
  // IMU-mode config
  temp = _i2c_bus->readByte(QMI8658C_ADDRESS, QMI8658C_CTRL7); // make sure not to disturb reserved bit values
  _i2c_bus->writeByte(QMI8658C_ADDRESS, QMI8658C_CTRL7, 0x03);  //  acc full scale and data rate
  
  //更新加速度计量程和采样速率
  temp = _i2c_bus->readByte(QMI8658C_ADDRESS, QMI8658C_CTRL2); // make sure not to disturb reserved bit values
  _i2c_bus->writeByte(QMI8658C_ADDRESS, QMI8658C_CTRL2, AODR | Ascale << 4);  //  acc full scale and data rate

  //更新陀螺仪量程和采样速率
  temp = _i2c_bus->readByte(QMI8658C_ADDRESS, QMI8658C_CTRL3);
  _i2c_bus->writeByte(QMI8658C_ADDRESS, QMI8658C_CTRL3, GODR | Gscale << 4); // gyro full scale and data rate

  //低通滤波设置
  _i2c_bus->writeByte(QMI8658C_ADDRESS, QMI8658C_CTRL5, 0x11); // gyro full scale and data rate

  
  // // first config
  // uint8_t temp = _i2c_bus->readByte(QMI8658C_ADDRESS, QMI8658C_CTRL1); // make sure not to disturb reserved bit values
  // _i2c_bus->writeByte(QMI8658C_ADDRESS, QMI8658C_CTRL1, temp | 0x60);  //  acc full scale and data rate
  
  // // IMU-mode config
  // temp = _i2c_bus->readByte(QMI8658C_ADDRESS, QMI8658C_CTRL7); // make sure not to disturb reserved bit values
  // _i2c_bus->writeByte(QMI8658C_ADDRESS, QMI8658C_CTRL7, temp | 0x03);  //  acc full scale and data rate
  
  // //更新加速度计量程和采样速率
  // temp = _i2c_bus->readByte(QMI8658C_ADDRESS, QMI8658C_CTRL2); // make sure not to disturb reserved bit values
  // _i2c_bus->writeByte(QMI8658C_ADDRESS, QMI8658C_CTRL2, temp | AODR | Ascale << 5);  //  acc full scale and data rate

  // //更新陀螺仪量程和采样速率
  // temp = _i2c_bus->readByte(QMI8658C_ADDRESS, QMI8658C_CTRL3);
  // _i2c_bus->writeByte(QMI8658C_ADDRESS, QMI8658C_CTRL3, temp | GODR | Gscale << 5); // gyro full scale and data rate
//
//  //更新磁力计量程和采样速率
//  temp = _i2c_bus->readByte(QMI8658C_ADDRESS, QMI8658C_CTRL4);
//  _i2c_bus->writeByte(QMI8658C_ADDRESS, QMI8658C_CTRL4, temp | MODR | 0 << 4); // mag full scale and data rat
}

/*
void QMI8658C::selfTest()
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int16_t accelPTest[3] = {0, 0, 0}, accelNTest[3] = {0, 0, 0}, gyroPTest[3] = {0, 0, 0}, gyroNTest[3] = {0, 0, 0};
  int16_t accelNom[3] = {0, 0, 0}, gyroNom[3] = {0, 0, 0};

  readData(temp);
  accelNom[0] = temp[4];
  accelNom[1] = temp[5];
  accelNom[2] = temp[6];
  gyroNom[0]  = temp[1];
  gyroNom[1]  = temp[2];
  gyroNom[2]  = temp[3];

  _i2c_bus->writeByte(QMI8658C_ADDRESS, QMI8658C_CTRL5_C, 0x01); // positive accel self test
  delay(100); // let accel respond
  readData(temp);
  accelPTest[0] = temp[4];
  accelPTest[1] = temp[5];
  accelPTest[2] = temp[6];

  _i2c_bus->writeByte(QMI8658C_ADDRESS, QMI8658C_CTRL5_C, 0x03); // negative accel self test
  delay(100); // let accel respond
  readData(temp);
  accelNTest[0] = temp[4];
  accelNTest[1] = temp[5];
  accelNTest[2] = temp[6];

  _i2c_bus->writeByte(QMI8658C_ADDRESS, QMI8658C_CTRL5_C, 0x04); // positive gyro self test
  delay(100); // let gyro respond
  readData(temp);
  gyroPTest[0] = temp[1];
  gyroPTest[1] = temp[2];
  gyroPTest[2] = temp[3];

  _i2c_bus->writeByte(QMI8658C_ADDRESS, QMI8658C_CTRL5_C, 0x0C); // negative gyro self test
  delay(100); // let gyro respond
  readData(temp);
  gyroNTest[0] = temp[1];
  gyroNTest[1] = temp[2];
  gyroNTest[2] = temp[3];

  _i2c_bus->writeByte(QMI8658C_ADDRESS, QMI8658C_CTRL5_C, 0x00); // normal mode
  delay(100); // let accel and gyro respond

  Serial.println("Accel Self Test:");
  Serial.print("+Ax results:"); Serial.print(  (accelPTest[0] - accelNom[0]) * _aRes * 1000.0); Serial.println(" mg");
  Serial.print("-Ax results:"); Serial.println((accelNTest[0] - accelNom[0]) * _aRes * 1000.0);
  Serial.print("+Ay results:"); Serial.println((accelPTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("-Ay results:"); Serial.println((accelNTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("+Az results:"); Serial.println((accelPTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.print("-Az results:"); Serial.println((accelNTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.println("Should be between 90 and 1700 mg");

  Serial.println("Gyro Self Test:");
  Serial.print("+Gx results:"); Serial.print((gyroPTest[0] - gyroNom[0]) * _gRes); Serial.println(" dps");
  Serial.print("-Gx results:"); Serial.println((gyroNTest[0] - gyroNom[0]) * _gRes);
  Serial.print("+Gy results:"); Serial.println((gyroPTest[1] - gyroNom[1]) * _gRes);
  Serial.print("-Gy results:"); Serial.println((gyroNTest[1] - gyroNom[1]) * _gRes);
  Serial.print("+Gz results:"); Serial.println((gyroPTest[2] - gyroNom[2]) * _gRes);
  Serial.print("-Gz results:"); Serial.println((gyroNTest[2] - gyroNom[2]) * _gRes);
  Serial.println("Should be between 20 and 80 dps");
  delay(2000);


}

*/
void QMI8658C::offsetBias(float * dest1, float * dest2)
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};

  Serial.println("Calculate accel and gyro offset biases: keep sensor flat and motionless!");
  delay(4000);

  for (int ii = 0; ii < 128; ii++)
  {
    readData(temp);
    sum[1] += temp[1];
    sum[2] += temp[2];
    sum[3] += temp[3];
    sum[4] += temp[4];
    sum[5] += temp[5];
    sum[6] += temp[6];
    delay(50);
  }

  dest1[0] = sum[1] * _aRes / 128.0f;
  dest1[1] = sum[2] * _aRes / 128.0f;
  dest1[2] = sum[3] * _aRes / 128.0f;
  dest2[0] = sum[4] * _gRes / 128.0f;
  dest2[1] = sum[5] * _gRes / 128.0f;
  dest2[2] = sum[6] * _gRes / 128.0f;

  if (dest1[0] > 0.8f)  {
    dest1[0] -= 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest1[0] < -0.8f) {
    dest1[0] += 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest1[1] > 0.8f)  {
    dest1[1] -= 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest1[1] < -0.8f) {
    dest1[1] += 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest1[2] > 0.8f)  {
    dest1[2] -= 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }
  if (dest1[2] < -0.8f) {
    dest1[2] += 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }

}


void QMI8658C::readData(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
//  for(int i=0;i<14;i++)
//  {
//    rawData[i] = _i2c_bus->readByte(QMI8658C_ADDRESS, QMI8658C_TEMP_DATA0+i);
//    }
  _i2c_bus->readBytes(QMI8658C_ADDRESS, QMI8658C_TEMP_DATA0, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
  destination[3] = ((int16_t)rawData[7] << 8) | rawData[6] ;
  destination[4] = ((int16_t)rawData[9] << 8) | rawData[8] ;
  destination[5] = ((int16_t)rawData[11] << 8) | rawData[10] ;
  destination[6] = ((int16_t)rawData[13] << 8) | rawData[12] ;  

}
