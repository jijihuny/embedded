#ifndef __MPU9250_H__
#define __MPU9250_H__
#include "SPI.h"

#define PI 3.14159

#define AFS_2G 0x00
#define AFS_4G 0x01
#define AFS_8G 0x02
#define AFS_16G 0x03

#define GFS_250DPS 0x00
#define GFS_500DPS 0x01
#define GFS_1000DPS 0x02
#define GFS_2000DPS 0x03

#define ACCEL_DIV 2.0f/32768.0f//ACCEL SCALE will be set as 2.0g
#define RADtoDEG  180.0f/PI
#define GYRO_DIV 250.0f/32768.0f//GYRO SCALE will be set as 250deg/sec
//constants

#define CONFIG (uint8_t) (0x1A)
#define GYRO_CONFIG (uint8_t) (0x1B)
#define ACCEL_CONFIG (uint8_t) (0x1C)
#define ACCEL_CONFIG2 (uint8_t) (0x1D)
//resistors for setting GYRO and ACCEL

#define ACCEL_XOUT_H (uint8_t) (0x3B)
#define ACCEL_XOUT_L (uint8_t) (0x3C)
#define ACCEL_YOUT_H (uint8_t) (0x3D)
#define ACCEL_YOUT_L (uint8_t) (0x3E)
#define ACCEL_ZOUT_H (uint8_t) (0x3F)
#define ACCEL_ZOUT_L (uint8_t) (0x40)
//resistors for getting data of GYRO

#define GYRO_XOUT_H (uint8_t) (0x43)
#define GYRO_XOUT_L (uint8_t) (0x44)
#define GYRO_YOUT_H (uint8_t) (0x45)
#define GYRO_YOUT_L (uint8_t) (0x46)
#define GYRO_ZOUT_H (uint8_t) (0x47)
#define GYRO_ZOUT_L (uint8_t) (0x48)
//resisotors for getting data of ACCEL

#define PWR_MGMT_1 (uint8_t) (0x6B)
#define PWR_MGMT_2 (uint8_t) (0x6C)
//POWER MANAGEMENT

#define INT_PIN_CFG (uint8_t) (0x37)
#define INT_ENABLE (uint8_t) (0x38)
#define INT_STATUS (uint8_t) (0x3A)
//INTERRUPT

#define FIFO_EN (uint8_t) (0x23)
#define FIFO_COUNT_H (uint8_t) (0x72)
#define FIFO_COUNT_L (uint8_t) (0x73)
#define FIFO_R_W (uint8_t) (0x74)
//FIFO

#define XG_OFFSET_H (uint8_t) (0x13)
#define XG_OFFSET_L (uint8_t) (0x14)
#define YG_OFFSET_H (uint8_t) (0x15)
#define YG_OFFSET_L (uint8_t) (0x16)
#define ZG_OFFSET_H (uint8_t) (0x17)
#define ZG_OFFSET_L (uint8_t) (0x18)
//GYROMETER offset resistor

#define XA_OFFSET_H (uint8_t) (0x77)
#define XA_OFFSET_L (uint8_t) (0x78)
#define YA_OFFSET_H (uint8_t) (0x7A)
#define YA_OFFSET_L (uint8_t) (0x7B)
#define ZA_OFFSET_H (uint8_t) (0x7C)
#define ZA_OFFSET_L (uint8_t) (0x7D)
//ACCELORMETER offset resistor

#define USER_CTRL (uint8_t) (0x6A)
// User control resistor
#define SMPLRT_DIV (uint8_t) (0x19)
//Sample rate divider


void calibrate_sensor(float* accel, float* gyro);
void MPU9250_init(uint8_t, uint8_t, uint8_t);
//void GetSensorData(float*);
void GetAccelData(float*);
void GetGyroData(float*);
//void GetMagData(float*);

float ComplimentaryFilter(float, float, float, float);



#endif