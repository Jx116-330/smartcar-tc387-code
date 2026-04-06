/*
 * icm42688.h
 *
 *  Created on: 2024��8��2��
 *      Author: ljk
 *      mail:983688746@qq.com
 */

#ifndef CODE_ICM42688_ICM42688_H_
#define CODE_ICM42688_ICM42688_H_
#include "zf_common_headfile.h"
/*********************************************************************************************************************
* File     : icm42688.h
* Brief    : ICM42688 driver header for TC387 reference project
*********************************************************************************************************************/
#define ICM42688_SPI            SPI_2
#define ICM42688_SPEED          3000000
#define ICM42688_SCK_Pin        SPI2_SCLK_P15_8
#define ICM42688_MOSI_Pin       SPI2_MOSI_P15_6
#define ICM42688_MISO_Pin       SPI2_MISO_P15_7
#define ICM42688_CS_Pin         SPI2_CS_P15_9

#define ICM42688_ID             0X47
#define WHO_AM_I                0XF500
#define READ_ACC_X_HIGH         0X9F

typedef enum
{
    GYRO_15_625DPS,
    GYRO_31_25DPS,
    GYRO_62_5DPS,
    GYRO_125DPS,
    GYRO_250DPS,
    GYRO_500DPS,
    GYRO_1000DPS,
    GYRO_2000DPS,
}GYRO_FSR;

typedef enum
{
    GYRO_ODR_12_5HZ,
    GYRO_ODR_25HZ,
    GYRO_ODR_50HZ,
    GYRO_ODR_100HZ,
    GYRO_ODR_200HZ,
    GYRO_ODR_500HZ,
    GYRO_ODR_1000HZ,
    GYRO_ODR_2000HZ,
    GYRO_ODR_4000HZ,
    GYRO_ODR_8000HZ,
    GYRO_ODR_16000HZ,
    GYRO_ODR_32000HZ,
}GYRO_ODR;

typedef enum
{
    ACC_2G,
    ACC_4G,
    ACC_8G,
    ACC_16G,
}ACC_FSR;

typedef enum
{
    ACC_ODR_12_5HZ,
    ACC_ODR_25HZ,
    ACC_ODR_50HZ,
    ACC_ODR_100HZ,
    ACC_ODR_200HZ,
    ACC_ODR_500HZ,
    ACC_ODR_1000HZ,
    ACC_ODR_2000HZ,
    ACC_ODR_4000HZ,
    ACC_ODR_8000HZ,
    ACC_ODR_16000HZ,
    ACC_ODR_32000HZ,
}ACC_ODR;

typedef enum
{
    _1st,
    _2st,
    _3st,
}Filter_Order;

typedef enum
{
    Bandwidth_Factor_2,
    Bandwidth_Factor_4,
    Bandwidth_Factor_5,
    Bandwidth_Factor_8,
    Bandwidth_Factor_10,
    Bandwidth_Factor_16,
    Bandwidth_Factor_20,
    Bandwidth_Factor_40,
    Low_latency_1,
    Low_Latency_2,
}Bandwidth_Factor;

typedef enum
{
    Bias_On_Chip_On,
    Bias_On_Chip_Off,
}Bias_On_Chip;

typedef struct
{
    GYRO_FSR GYRO_FSR;
    GYRO_ODR GYRO_ODR;
    ACC_FSR ACC_FSR;
    ACC_ODR ACC_ODR;
    Filter_Order Gyro_Filter_Order;
    Bandwidth_Factor Gyro_Bandwidth_Factor;
    Filter_Order Acc_Filter_Order;
    Bandwidth_Factor Acc_Bandwidth_Factor;
    Bias_On_Chip Bias_Option;
}ICM42688_CONFIG_STRUCT;

typedef struct
{
    int16 acc_x_lsb;
    int16 acc_y_lsb;
    int16 acc_z_lsb;
    int16 gyro_x_lsb;
    int16 gyro_y_lsb;
    int16 gyro_z_lsb;
}ICM42688_RAW_DATA;

typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
}ICM42688_real_data;

#define SENSITIVITY_ICM42688_GYRO_15_625dps      2097.2f
#define SENSITIVITY_ICM42688_GYRO_31_25dps       1048.6f
#define SENSITIVITY_ICM42688_GYRO_62_5dps        524.3f
#define SENSITIVITY_ICM42688_GYRO_125dps         262.1f
#define SENSITIVITY_ICM42688_GYRO_250dps         131.0f
#define SENSITIVITY_ICM42688_GYRO_500dps         65.5f
#define SENSITIVITY_ICM42688_GYRO_1000dps        32.8f
#define SENSITIVITY_ICM42688_GYRO_2000dps        16.4f

#define SENSITIVITY_ICM42688_ACC_2G             16384.0f
#define SENSITIVITY_ICM42688_ACC_4G             8192.0f
#define SENSITIVITY_ICM42688_ACC_8G             4096.0f
#define SENSITIVITY_ICM42688_ACC_16G            2048.0f

extern ICM42688_CONFIG_STRUCT ICM42688_CONFIG;
extern ICM42688_RAW_DATA ICM42688_RAW;
extern ICM42688_real_data ICM42688;

void icm42688_init(ICM42688_CONFIG_STRUCT *ICM42688_CONFIG);
void ICM42688_Get_Data(void);

#endif /* CODE_ICM42688_ICM42688_H_ */
