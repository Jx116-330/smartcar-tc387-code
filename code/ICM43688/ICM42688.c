/*
 * icm42688.c
 *
 *  Created on: 2025��2��20��
 *      Author: 35430
 */
#include "icm42688.h"

float icm42688_acc_x  = 0, icm42688_acc_y  = 0, icm42688_acc_z  = 0;    // ICM42688���ٶȼ�����
float icm42688_gyro_x = 0, icm42688_gyro_y = 0, icm42688_gyro_z = 0;    // ICM42688�Ǽ��ٶ�����

// SPIЭ���д�����궨��,��ѧ�������޸�
#if ICM42688_HARD_SPI
#define ICM42688_Write_Reg(reg, data)       spi_write_8bit_register(ICM42688_SPI, reg, data);
#define ICM42688_Read_Regs(reg, data,num)   spi_read_8bit_registers(ICM42688_SPI, reg | 0x80, data, num);
#else
static SOFT_SPI_struct ICM42688_SPI;
#define ICM42688_Write_Reg(reg, data)       write_8bitreg_soft_spi(&ICM42688_SPI, reg, data);
#define ICM42688_Read_Regs(reg, data,num)   read_8bitregs_soft_spi(&ICM42688_SPI, reg | 0x80, data, num);
#endif

static float icm42688_acc_inv = 1, icm42688_gyro_inv = 1;               // ����ת��Ϊʵ���������ݵ�ת��ϵ��

/**
*
* @brief    ICM42688�����ǳ�ʼ��
* @param
* @return   void
* @notes    �û�����
* Example:  Init_ICM42688();
*
**/
void Init_ICM42688(void)
{
    // SPI��ʼ��
#if ICM42688_HARD_SPI
    spi_init(ICM42688_SPI, SPI_MODE0, ICM42688_SPI_SPEED, ICM42688_SPC_PIN, ICM42688_SDI_PIN, ICM42688_SDO_PIN, SPI_CS_NULL);
#else
    init_soft_spi (&ICM42688_SPI, 0, ICM42688_SPC_MODULE, ICM42688_SPC_PIN, ICM42688_SDI_MODULE, ICM42688_SDI_PIN, ICM42688_SDO_MODULE, ICM42688_SDO_PIN);
#endif

    gpio_init(ICM42688_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    // ��ȡ�������ͺ��������Լ�
    unsigned char model = 0xff;
    while(1)
    {
        unsigned char time = 50;
        Read_Datas_ICM42688(ICM42688_WHO_AM_I, &model, 1); // ��оƬID

        if(model == 0x47)
        {

            // ICM42688,71
            break;
        }
        else
        {
            system_delay_ms(10);
            time--;
            if(time < 0)
            {

                // ��������ԭ�������¼���
                // ICM42688����,������µĸ��ʼ���
                // ���ߴ������û�нӺ�
            }
        }
    }
    Write_Data_ICM42688(ICM42688_PWR_MGMT0, 0x00);      // ��λ�豸
    system_delay_ms(10);                              // ������PWR��MGMT0�Ĵ�����200us�ڲ������κζ�д�Ĵ����Ĳ���

    // ����ICM42688���ٶȼƺ������ǵ����̺��������
    Set_LowpassFilter_Range_ICM42688(ICM42688_AFS_8G, ICM42688_AODR_32000HZ, ICM42688_GFS_2000DPS, ICM42688_GODR_32000HZ);
    Write_Data_ICM42688(ICM42688_PWR_MGMT0, 0x0f);      // ����GYRO_MODE,ACCEL_MODEΪ������ģʽ
    system_delay_ms(10);
}

/**
*
* @brief    ���ICM42688�����Ǽ��ٶ�
* @param
* @return   void
* @notes    ��λ:g(m/s^2),�û�����
* Example:  Get_Acc_ICM42688();
*
**/
void Get_Acc_ICM42688(void)
{
    unsigned char data[6];
    Read_Datas_ICM42688(ICM42688_ACCEL_DATA_X1, data, 6);
    icm42688_acc_x = icm42688_acc_inv * (short int)(((short int)data[0] << 8) | data[1]);
    icm42688_acc_y = icm42688_acc_inv * (short int)(((short int)data[2] << 8) | data[3]);
    icm42688_acc_z = icm42688_acc_inv * (short int)(((short int)data[4] << 8) | data[5]);
}

/**
*
* @brief    ���ICM42688�����ǽǼ��ٶ�
* @param
* @return   void
* @notes    ��λΪ:��/s,�û�����
* Example:  Get_Gyro_ICM42688();
*
**/
void Get_Gyro_ICM42688(void)
{
    unsigned char data[6];
    Read_Datas_ICM42688(ICM42688_GYRO_DATA_X1, data, 6);
    icm42688_gyro_x = icm42688_gyro_inv * (short int)(((short int)data[0] << 8) | data[1]);
    icm42688_gyro_y = icm42688_gyro_inv * (short int)(((short int)data[2] << 8) | data[3]);
    icm42688_gyro_z = icm42688_gyro_inv * (short int)(((short int)data[4] << 8) | data[5]);
}

/**
*
* @brief    ����ICM42688�����ǵ�ͨ�˲�������������
* @param    afs                 // ���ٶȼ�����,����dmx_icm42688.h�ļ���ö�ٶ����в鿴
* @param    aodr                // ���ٶȼ��������,����dmx_icm42688.h�ļ���ö�ٶ����в鿴
* @param    gfs                 // ����������,����dmx_icm42688.h�ļ���ö�ٶ����в鿴
* @param    godr                // �������������,����dmx_icm42688.h�ļ���ö�ٶ����в鿴
* @return   void
* @notes    ICM42688.c�ļ��ڲ�����,�û�������ó���
* Example:  Set_LowpassFilter_Range_ICM42688(ICM42688_AFS_16G,ICM42688_AODR_32000HZ,ICM42688_GFS_2000DPS,ICM42688_GODR_32000HZ);
*
**/
void Set_LowpassFilter_Range_ICM42688(enum icm42688_afs afs, enum icm42688_aodr aodr, enum icm42688_gfs gfs, enum icm42688_godr godr)
{
    Write_Data_ICM42688(ICM42688_ACCEL_CONFIG0, (afs << 5) | (aodr + 1));   // ��ʼ��ACCEL���̺��������(p77)
    Write_Data_ICM42688(ICM42688_GYRO_CONFIG0, (gfs << 5) | (godr + 1));    // ��ʼ��GYRO���̺��������(p76)

    switch(afs)
    {
    case ICM42688_AFS_2G:
        icm42688_acc_inv = 2 / 32768.0f;             // ���ٶȼ�����Ϊ:��2g
        break;
    case ICM42688_AFS_4G:
        icm42688_acc_inv = 4 / 32768.0f;             // ���ٶȼ�����Ϊ:��4g
        break;
    case ICM42688_AFS_8G:
        icm42688_acc_inv = 8 / 32768.0f;             // ���ٶȼ�����Ϊ:��8g
        break;
    case ICM42688_AFS_16G:
        icm42688_acc_inv = 16 / 32768.0f;            // ���ٶȼ�����Ϊ:��16g
        break;
    default:
        icm42688_acc_inv = 1;                           // ��ת��Ϊʵ������
        break;
    }
    switch(gfs)
    {
    case ICM42688_GFS_15_625DPS:
        icm42688_gyro_inv = 15.625f / 32768.0f;         // ����������Ϊ:��15.625dps
        break;
    case ICM42688_GFS_31_25DPS:
        icm42688_gyro_inv = 31.25f / 32768.0f;          // ����������Ϊ:��31.25dps
        break;
    case ICM42688_GFS_62_5DPS:
        icm42688_gyro_inv = 62.5f / 32768.0f;           // ����������Ϊ:��62.5dps
        break;
    case ICM42688_GFS_125DPS:
        icm42688_gyro_inv = 125.0f / 32768.0f;          // ����������Ϊ:��125dps
        break;
    case ICM42688_GFS_250DPS:
        icm42688_gyro_inv = 250.0f / 32768.0f;          // ����������Ϊ:��250dps
        break;
    case ICM42688_GFS_500DPS:
        icm42688_gyro_inv = 500.0f / 32768.0f;          // ����������Ϊ:��500dps
        break;
    case ICM42688_GFS_1000DPS:
        icm42688_gyro_inv = 1000.0f / 32768.0f;         // ����������Ϊ:��1000dps
        break;
    case ICM42688_GFS_2000DPS:
        icm42688_gyro_inv = 2000.0f / 32768.0f;         // ����������Ϊ:��2000dps
        break;
    default:
        icm42688_gyro_inv = 1;                          // ��ת��Ϊʵ������
        break;
    }
}

/**
*
* @brief    ICM42688������д����
* @param    reg                 �Ĵ���
* @param    data                ��Ҫд���üĴ���������
* @return   void
* @notes    ICM42688.c�ļ��ڲ�����,�û�������ó���
* Example:  Write_Reg_ICM42688(0X00,0X00);
*
**/
static void Write_Data_ICM42688(unsigned char reg, unsigned char data)
{
    ICM42688_CS_LEVEL(0);
    ICM42688_Write_Reg(reg, data);
    ICM42688_CS_LEVEL(1);
}

/**
*
* @brief    ICM42688�����Ƕ�����
* @param    reg                 �Ĵ���
* @param    data                �Ѷ��������ݴ���data
* @param    num                 ���ݸ���
* @return   void
* @notes    ICM42688.c�ļ��ڲ�����,�û�������ó���
* Example:  ICM42688_Read_Datas(0X00,0X00,1);
*
**/
static void Read_Datas_ICM42688(unsigned char reg, unsigned char *data, unsigned int num)
{
    ICM42688_CS_LEVEL(0);
    ICM42688_Read_Regs(reg, data, num);
    ICM42688_CS_LEVEL(1);
}

