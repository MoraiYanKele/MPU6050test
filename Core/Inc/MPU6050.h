#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "i2c.h"
#include "main.h"
#include "usart.h"
#include <stdio.h>

#define DEFAULT_MPU_HZ  (20)

#define MPU6050ADDRESS    0xD1  //mpu6050地址 
#define MPU6050_PWE_MGMT1  0xD1  //电源管理寄存器1

int MPU6050_DMP_Init();
int MPU6050_DMP_GetData(float *pitch, float *roll, float *yaw);



void MPU6050_Init();
// float ReadAccel_x(uint8_t *MPUdata);
// float ReadAccel_y(uint8_t *MPUdata);
// float ReadAccel_z(uint8_t *MPUdata);
// float ReadGyro_x(uint8_t *MPUdata);
// float ReadGyro_y(uint8_t *MPUdata);
// float ReadGyro_z(uint8_t *MPUdata);

#endif