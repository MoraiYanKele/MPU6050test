#include <math.h>
#include "MPU6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#define Q30     (1073741824.0f)     //Q30格式， 2的30次方


struct platform_data_s {
    signed char orientation[9];
};

static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;		// error
    return b;
}

unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{

    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}
static int run_self_test(void)
{
    int result;
    long gyro[3], accel[3];
    result = mpu_run_self_test(gyro, accel);
    if (result == 0x3) {
	// MPL_LOGI("Passed!\n");
    // MPL_LOGI("accel: %7.4f %7.4f %7.4f\n",
    //             accel[0]/65536.f,
    //             accel[1]/65536.f,
    //             accel[2]/65536.f);
    // MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n",
    //             gyro[0]/65536.f,
    //             gyro[1]/65536.f,
    //             gyro[2]/65536.f);
    unsigned short accel_sens;
    float gyro_sens;

    mpu_get_accel_sens(&accel_sens);
    accel[0] *= accel_sens;
    accel[1] *= accel_sens;
    accel[2] *= accel_sens;
    dmp_set_accel_bias(accel);
    mpu_get_gyro_sens(&gyro_sens);
    gyro[0] = (long) (gyro[0] * gyro_sens);
    gyro[1] = (long) (gyro[1] * gyro_sens);
    gyro[2] = (long) (gyro[2] * gyro_sens);
    dmp_set_gyro_bias(gyro);
    }
    else
    {
        return -1;
    }
    return 0;
}


/**
 * @brief  MPU6050DMP库初始化
 * @retval 各种情况对应的返回值
 * @note   NULL
 */
int MPU6050_DMP_Init()
{
    int result;
    struct int_param_s int_param;
    result = mpu_init(&int_param);  //mpu初始化
    if (result != 0)
    {
        return -1;
    }
      
    result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL); //设置传感器
    if (result != 0)
    {
        return -2;
    }
    result = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);  //设置fifo
    if (result != 0)
    {
        return -3;
    }
    result = mpu_set_sample_rate(DEFAULT_MPU_HZ);   //设置采样率
    if (result != 0)
    {
        return -4;
    }
    result = dmp_load_motion_driver_firmware(); //加载dmp固件
    if (result != 0)
    {
        return -5;
    }
    result = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation)); //设置陀螺仪方向
    if (result != 0)
    {
        return -6;
    }

     
    result = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                                DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                                DMP_FEATURE_GYRO_CAL);  //设置dmp功能
    if (result != 0)
    {
        return -7;
    }
    result = dmp_set_fifo_rate(DEFAULT_MPU_HZ);     //设置输出速率
    if (result != 0)
    {
        return -8;
    }

    // result = run_self_test();   //自检
    // if (result != 0)
    // {
    //     return -9;
    // }

    result = mpu_set_dmp_state(1); //使能dmp
    if (result != 0)
    {
        return -10;
    }
    

    return 0;
}



/**
 * @brief  MPU6050初始化，非DMP库
 * @note   数据存在杂项，建议DMP库
 */
void MPU6050_Init()
{
    uint8_t address = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050ADDRESS, 0x6B, I2C_MEMADD_SIZE_8BIT, &address, 1, 0xff);
    address = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050ADDRESS, 0x19, I2C_MEMADD_SIZE_8BIT, &address, 1, 0xff);
    address = 0x06;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050ADDRESS, 0x1A, I2C_MEMADD_SIZE_8BIT, &address, 1, 0xff);
    address = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050ADDRESS, 0x1B, I2C_MEMADD_SIZE_8BIT, &address, 1, 0xff);
    address = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050ADDRESS, 0x1C, I2C_MEMADD_SIZE_8BIT, &address, 1, 0xff);
    
}


/**
 * @brief  
 * @param  
 * @retval NULL
 * @note   NULL
 */
float ReadAccel_x(uint8_t *MPUdata)
{
    short accelOutput_x;
    accelOutput_x = MPUdata[0] << 8 | MPUdata[1];
    return (float)accelOutput_x / 16384;
}

float ReadAccel_y(uint8_t *MPUdata)
{
    short accelOutput_y;
    accelOutput_y = MPUdata[2] << 8 | MPUdata[3];
    return (float)accelOutput_y / 16384;
}

float ReadAccel_z(uint8_t *MPUdata)
{
    short accelOutput_z;
    accelOutput_z = MPUdata[4] << 8 | MPUdata[5];
    return (float)accelOutput_z / 16384;
}

float ReadGyro_x(uint8_t *MPUdata)
{
    short gyroOutput_x;
    gyroOutput_x = MPUdata[8] << 8 | MPUdata[9];
    return (float)gyroOutput_x / 65.5;
}

float ReadGyro_y(uint8_t *MPUdata)
{
    short gyroOutput_y;
    gyroOutput_y = MPUdata[10] << 8 | MPUdata[11];
    return (float)gyroOutput_y / 65.5;
}

float ReadGyro_z(uint8_t *MPUdata)
{
    short gyroOutput_z;
    gyroOutput_z = MPUdata[12] << 8 | MPUdata[13];
    return (float)gyroOutput_z / 65.5;
}



/**
 * @brief  MPU6050DMP库读取数据
 * @param  float *pitch：pitch角存储变量的地址
 * @param  float *roll： roll角存储变量的地址
 * @param  float *yaw： yaw角存储变量的地址
 * @retval 0：获取数据成功
 * @retval -1：获取数据失败
 * @note   yaw角存在偏移
 */
int MPU6050_DMP_GetData(float *pitch, float *roll, float *yaw)
{
    float q0, q1, q2, q3;
    short gyro[3];
    short accel[3]; 
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more))
    {
        return -1;
    }

    if (sensors & INV_WXYZ_QUAT)
    {
        q0 = quat[0] / Q30;
        q1 = quat[1] / Q30;
        q2 = quat[2] / Q30;
        q3 = quat[3] / Q30;
    }

    *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
    *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;
    *yaw   = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;

    return 0;
}