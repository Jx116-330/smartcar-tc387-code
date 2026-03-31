/*********************************************************************************************************************
* TC387 Opensourec Library 即（TC387 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC387 开源库的一部分
*
* TC387 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          zf_device_wifi_spi
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC387QP
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2024-01-18        SeekFree            first version
*********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   RST                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_RST_PIN 宏定义
*                   INT                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_INT_PIN 宏定义
*                   CS                  查看 zf_device_wifi_spi.h 中 WIFI_SPI_CS_PIN 宏定义
*                   MISO                查看 zf_device_wifi_spi.h 中 WIFI_SPI_MISO_PIN 宏定义
*                   SCK                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_SCK_PIN 宏定义
*                   MOSI                查看 zf_device_wifi_spi.h 中 WIFI_SPI_MOSI_PIN 宏定义
*                   5V                  5V 电源
*                   GND                 电源地
*                   其余引脚悬空
*                   ------------------------------------
*********************************************************************************************************************/

#ifndef _zf_device_wifi_spi_h
#define _zf_device_wifi_spi_h

#include "zf_common_typedef.h"

#define WIFI_SPI_INDEX              (SPI_4             )
#define WIFI_SPI_SPEED              (30 * 1000 * 1000  )
#define WIFI_SPI_SCK_PIN            (SPI4_SCLK_P22_3   )
#define WIFI_SPI_MOSI_PIN           (SPI4_MOSI_P22_0   )
#define WIFI_SPI_MISO_PIN           (SPI4_MISO_P22_1   )
#define WIFI_SPI_CS_PIN             (P22_2             )
#define WIFI_SPI_INT_PIN            (P15_8             )
#define WIFI_SPI_RST_PIN            (P23_1             )

#define WIFI_SPI_RECVIVE_FIFO_SIZE  (1024)
#define WIFI_SPI_READ_TRANSFER      (1)
#define WIFI_SPI_AUTO_CONNECT       (0)

#if (WIFI_SPI_AUTO_CONNECT > 2)
#error "WIFI_SPI_AUTO_CONNECT 的值只能为 [0,1,2]"
#else
#define WIFI_SPI_TARGET_IP          "192.168.137.1"
#define WIFI_SPI_TARGET_PORT        "8086"
#define WIFI_SPI_LOCAL_PORT         "6666"
#endif

#define WIFI_SPI_RECVIVE_SIZE       (32)
#define WIFI_SPI_TRANSFER_SIZE      (4088)

typedef enum
{
    WIFI_SPI_INVALID1               = 0x00,
    WIFI_SPI_RESET                  = 0x01,
    WIFI_SPI_DATA                   = 0x02,
    WIFI_SPI_UDP_SEND               = 0x03,
    WIFI_SPI_CLOSE_SOCKET           = 0x04,
    WIFI_SPI_SET_WIFI_INFORMATION   = 0x10,
    WIFI_SPI_SET_SOCKET_INFORMATION = 0x11,
    WIFI_SPI_GET_VERSION            = 0x20,
    WIFI_SPI_GET_MAC_ADDR           = 0x21,
    WIFI_SPI_GET_IP_ADDR            = 0x22,
    WIFI_SPI_REPLY_OK               = 0x80,
    WIFI_SPI_REPLY_ERROR            = 0x81,
    WIFI_SPI_REPLY_DATA_START       = 0x90,
    WIFI_SPI_REPLY_DATA_END         = 0x91,
    WIFI_SPI_REPLY_VERSION          = 0xA0,
    WIFI_SPI_REPLY_MAC_ADDR         = 0xA1,
    WIFI_SPI_REPLY_IP_ADDR          = 0xA2,
    WIFI_SPI_INVALID2               = 0xFF
} wifi_spi_packets_command_enum;

typedef enum
{
    WIFI_SPI_IDLE,
    WIFI_SPI_BUSY,
} wifi_spi_state_enum;

typedef struct
{
    uint8   command;
    uint8   reserve;
    uint16  length;
} wifi_spi_head_struct;

typedef struct
{
    wifi_spi_head_struct  head;
    uint8 buffer[WIFI_SPI_RECVIVE_SIZE];
} wifi_spi_packets_struct;

extern char wifi_spi_version[12];
extern char wifi_spi_mac_addr[20];
extern char wifi_spi_ip_addr_port[25];

uint8  wifi_spi_wifi_connect        (char *wifi_ssid, char *pass_word);
uint8  wifi_spi_socket_connect      (char *transport_type, char *ip_addr, char *port, char *local_port);
uint8  wifi_spi_socket_disconnect   (void);
uint8  wifi_spi_udp_send_now        (void);
uint32 wifi_spi_send_buffer         (const uint8 *buff, uint32 length);
uint32 wifi_spi_read_buffer         (uint8 *buffer, uint32 length);
uint8  wifi_spi_init                (char *wifi_ssid, char *pass_word);

#endif
