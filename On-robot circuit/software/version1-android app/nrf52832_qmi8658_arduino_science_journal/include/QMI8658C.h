/* 05/03/2022
    Created by ZenanLin
    Library may be used freely and without limit with attribution.
*/

#ifndef QMI8658C_h
#define QMI8658C_h

#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"

/* QMI8658C registers
https://cloud.tsinghua.edu.cn/f/8e49ffa7e3864bd1b7cc/?dl=1
*/
// Bank 0
#define QMI8658C_DEVICE_CONFIG             0x60// *
#define QMI8658C_DRIVE_CONFIG              0x13
#define QMI8658C_INT_CONFIG                0x14
#define QMI8658C_FIFO_CONFIG               0x16

// read out registor
#define QMI8658C_TEMP_DATA1                0x34 //*H
#define QMI8658C_TEMP_DATA0                0x33 //*L
#define QMI8658C_ACCEL_DATA_X1             0x36 //*H
#define QMI8658C_ACCEL_DATA_X0             0x35 //*L
#define QMI8658C_ACCEL_DATA_Y1             0x38 //*H
#define QMI8658C_ACCEL_DATA_Y0             0x37 //*L
#define QMI8658C_ACCEL_DATA_Z1             0x3A //*H
#define QMI8658C_ACCEL_DATA_Z0             0x39 //*L
#define QMI8658C_GYRO_DATA_X1              0x3C //*H
#define QMI8658C_GYRO_DATA_X0              0x3B //*L
#define QMI8658C_GYRO_DATA_Y1              0x3E //*H
#define QMI8658C_GYRO_DATA_Y0              0x3D //*L
#define QMI8658C_GYRO_DATA_Z1              0x40 //*H
#define QMI8658C_GYRO_DATA_Z0              0x3F //*L
#define QMI8658C_MAG_DATA_X1               0x42 //*H
#define QMI8658C_MAG_DATA_X0               0x41 //*L
#define QMI8658C_MAG_DATA_Y1               0x44 //*H
#define QMI8658C_MAG_DATA_Y0               0x43 //*L
#define QMI8658C_MAG_DATA_Z1               0x46 //*H
#define QMI8658C_MAG_DATA_Z0               0x45 //*L

// 9轴传感器配置相关寄存器
#define QMI8658C_CTRL7                     0x08 //*
#define QMI8658C_CTRL1                     0x02 //*
#define QMI8658C_CTRL2                     0x03 //*
#define QMI8658C_CTRL3                     0x04 //*
#define QMI8658C_CTRL4                     0x05 //*
#define QMI8658C_CTRL5                     0x06 //*

#define QMI8658C_TMST_FSYNCH               0x2B
#define QMI8658C_TMST_FSYNCL               0x2C
#define QMI8658C_INT_STATUS                0x2D
#define QMI8658C_FIFO_COUNTH               0x2E
#define QMI8658C_FIFO_COUNTL               0x2F
#define QMI8658C_FIFO_DATA                 0x30
#define QMI8658C_APEX_DATA0                0x31
#define QMI8658C_APEX_DATA1                0x32
#define QMI8658C_APEX_DATA2                0x33
#define QMI8658C_APEX_DATA3                0x34
#define QMI8658C_APEX_DATA4                0x35
#define QMI8658C_APEX_DATA5                0x36
#define QMI8658C_INT_STATUS2               0x37
#define QMI8658C_INT_STATUS3               0x38
#define QMI8658C_SIGNAL_PATH_RESET         0x4B
#define QMI8658C_INTF_CONFIG0              0x4C
#define QMI8658C_INTF_CONFIG1              0x4D
#define QMI8658C_PWR_MGMT0                 0x4E
#define QMI8658C_GYRO_CONFIG0              0x4F
#define QMI8658C_ACCEL_CONFIG0             0x50
#define QMI8658C_GYRO_CONFIG1              0x51
#define QMI8658C_GYRO_ACCEL_CONFIG0        0x52
#define QMI8658C_ACCEL_CONFIG1             0x53
#define QMI8658C_TMST_CONFIG               0x54
#define QMI8658C_APEX_CONFIG0              0x56
#define QMI8658C_SMD_CONFIG                0x57
#define QMI8658C_FIFO_CONFIG1              0x5F
#define QMI8658C_FIFO_CONFIG2              0x60
#define QMI8658C_FIFO_CONFIG3              0x61
#define QMI8658C_FSYNC_CONFIG              0x62
#define QMI8658C_INT_CONFIG0               0x63
#define QMI8658C_INT_CONFIG1               0x64
#define QMI8658C_INT_SOURCE0               0x65
#define QMI8658C_INT_SOURCE1               0x66
#define QMI8658C_INT_SOURCE3               0x68
#define QMI8658C_INT_SOURCE4               0x69
#define QMI8658C_FIFO_LOST_PKT0            0x6C
#define QMI8658C_FIFO_LOST_PKT1            0x6D
#define QMI8658C_SELF_TEST_CONFIG          0x70
#define QMI8658C_WHO_AM_I                  0x00 // *
#define QMI8658C_REG_BANK_SEL              0x76

// Bank 1
#define QMI8658C_SENSOR_CONFIG0            0x03
#define QMI8658C_GYRO_CONFIG_STATIC2       0x0B
#define QMI8658C_GYRO_CONFIG_STATIC3       0x0C
#define QMI8658C_GYRO_CONFIG_STATIC4       0x0D
#define QMI8658C_GYRO_CONFIG_STATIC5       0x0E
#define QMI8658C_GYRO_CONFIG_STATIC6       0x0F
#define QMI8658C_GYRO_CONFIG_STATIC7       0x10
#define QMI8658C_GYRO_CONFIG_STATIC8       0x11
#define QMI8658C_GYRO_CONFIG_STATIC9       0x12
#define QMI8658C_GYRO_CONFIG_STATIC10      0x13
#define QMI8658C_XG_ST_DATA                0x5F
#define QMI8658C_YG_ST_DATA                0x60
#define QMI8658C_ZG_ST_DATA                0x61
#define QMI8658C_TMSTVAL0                  0x62
#define QMI8658C_TMSTVAL1                  0x63
#define QMI8658C_TMSTVAL2                  0x64
#define QMI8658C_INTF_CONFIG4              0x7A
#define QMI8658C_INTF_CONFIG5              0x7B
#define QMI8658C_INTF_CONFIG6              0x7C

// Bank 2
#define QMI8658C_ACCEL_CONFIG_STATIC2      0x03
#define QMI8658C_ACCEL_CONFIG_STATIC3      0x04
#define QMI8658C_ACCEL_CONFIG_STATIC4      0x05
#define QMI8658C_XA_ST_DATA                0x3B
#define QMI8658C_YA_ST_DATA                0x3C
#define QMI8658C_ZA_ST_DATA                0x3D

// Bank 4
#define QMI8658C_GYRO_ON_OFF_CONFIG        0x0E
#define QMI8658C_APEX_CONFIG1              0x40
#define QMI8658C_APEX_CONFIG2              0x41
#define QMI8658C_APEX_CONFIG3              0x42
#define QMI8658C_APEX_CONFIG4              0x43
#define QMI8658C_APEX_CONFIG5              0x44
#define QMI8658C_APEX_CONFIG6              0x45
#define QMI8658C_APEX_CONFIG7              0x46
#define QMI8658C_APEX_CONFIG8              0x47
#define QMI8658C_APEX_CONFIG9              0x48
#define QMI8658C_ACCEL_WOM_X_THR           0x4A
#define QMI8658C_ACCEL_WOM_Y_THR           0x4B
#define QMI8658C_ACCEL_WOM_Z_THR           0x4C
#define QMI8658C_INT_SOURCE6               0x4D
#define QMI8658C_INT_SOURCE7               0x4E
#define QMI8658C_INT_SOURCE8               0x4F
#define QMI8658C_INT_SOURCE9               0x50
#define QMI8658C_INT_SOURCE10              0x51
#define QMI8658C_OFFSET_USER0              0x77
#define QMI8658C_OFFSET_USER1              0x78
#define QMI8658C_OFFSET_USER2              0x79
#define QMI8658C_OFFSET_USER3              0x7A
#define QMI8658C_OFFSET_USER4              0x7B
#define QMI8658C_OFFSET_USER5              0x7C
#define QMI8658C_OFFSET_USER6              0x7D
#define QMI8658C_OFFSET_USER7              0x7E
#define QMI8658C_OFFSET_USER8              0x7F

#define QMI8658C_ADDRESS                   0x6B   //* Address of QMI8658C accel/gyro when ADO = LOW

//* acc 量程
#define AFS_2G  0x00   // default
#define AFS_4G  0x01
#define AFS_8G  0x02
#define AFS_16G 0x03  

//* acc 采样速率
#define AODR_8000Hz   0x00 // default
#define AODR_4000Hz   0x01
#define AODR_2000Hz   0x02
#define AODR_1000Hz   0x03 
#define AODR_500Hz    0x04
#define AODR_250Hz    0x05
#define AODR_125Hz    0x06
#define AODR_62_5Hz   0x07
#define AODR_31_5Hz   0x08
#define AODR_128Hz    0x0C
#define AODR_21Hz     0x0D
#define AODR_11Hz     0x0E
#define AODR_3Hz      0x0F

//*gyro 量程
#define GFS_2048DPS   0x07
#define GFS_1024DPS   0x06
#define GFS_512DPS    0x05
#define GFS_256DPS    0x04
#define GFS_128DPS    0x03
#define GFS_64DPS     0x02
#define GFS_32DPS     0x01
#define GFS_16DPS     0x00 // default

//*gyro 采样速率
#define GODR_8000Hz   0x00 // default
#define GODR_4000Hz   0x01
#define GODR_2000Hz   0x02
#define GODR_1000Hz   0x03 
#define GODR_500Hz    0x04
#define GODR_250Hz    0x05
#define GODR_125Hz    0x06
#define GODR_62_5Hz   0x07
#define GODR_31_5Hz   0x08

class QMI8658C
{
  public:
  QMI8658C(I2Cdev* i2c_bus);
  float getAres(uint8_t Ascale);
  float getGres(uint8_t Gscale);
  uint8_t getChipID();
  void init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR);
  void offsetBias(float * dest1, float * dest2);
  void reset();
  void selfTest();
  void readData(int16_t * destination);
  uint8_t status();
  private:
  float _aRes, _gRes;
  I2Cdev* _i2c_bus;
};

#endif
