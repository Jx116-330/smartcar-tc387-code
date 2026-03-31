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
#include "stdio.h"
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"
#include "zf_device_type.h"

#include "zf_device_wifi_spi.h"

#define WIFI_CONNECT_TIME_OUT       10000       // 单位毫秒
#define SOCKET_CONNECT_TIME_OUT     50000       // 单位毫秒
#define OTHER_TIME_OUT              1000        // 单位毫秒

char wifi_spi_version[12];                      // 保存模块固件版本信息
char wifi_spi_mac_addr[20];                     // 保存模块MAC地址信息
char wifi_spi_ip_addr_port[25];                 // 保存模块IP地址与端口信息

static fifo_struct  wifi_spi_fifo;
static uint8        wifi_spi_buffer[WIFI_SPI_RECVIVE_FIFO_SIZE];
static volatile     wifi_spi_state_enum wifi_spi_mutex;
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     等待WIFI SPI就绪
// 参数说明     wait_time       最大等待时间 单位毫秒
// 返回参数     uint8           状态 0-成功 1-错误
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_wait_idle (uint32 wait_time)
{
    uint32 time = 0;
    
    wait_time = wait_time*100;
    while(0 == gpio_get_level(WIFI_SPI_INT_PIN))
    {
        system_delay_us(10);
        time++;
        if(wait_time <= time)
        {
            break;
        }
    }
    return (wait_time <= time);
}

static void wifi_spi_write (const uint8 *buffer1, uint16 length1, const uint8 *buffer2, uint16 length2)
{
    gpio_low(WIFI_SPI_CS_PIN);
    if(NULL != buffer1)
    {
        spi_write_8bit_array(WIFI_SPI_INDEX, buffer1, length1);
    }
    if(NULL != buffer2)
    {
        spi_write_8bit_array(WIFI_SPI_INDEX, buffer2, length2);
    }
    gpio_high(WIFI_SPI_CS_PIN);
}

static void wifi_spi_transfer_command (wifi_spi_packets_struct *packets, uint16 length)
{
    gpio_low(WIFI_SPI_CS_PIN);
    
    spi_transfer_8bit(WIFI_SPI_INDEX, (uint8 *)&(packets->head), (uint8 *)&(packets->head), sizeof(wifi_spi_head_struct));
    
    if(length)
    {
        spi_transfer_8bit(WIFI_SPI_INDEX, (const uint8 *)(packets->buffer), packets->buffer, length);
    }
    
    gpio_high(WIFI_SPI_CS_PIN);
}

static void wifi_spi_transfer_data (const uint8 *write_data, wifi_spi_packets_struct *read_data, uint16 length)
{
    gpio_low(WIFI_SPI_CS_PIN);
    
    read_data->head.command = WIFI_SPI_DATA;
    read_data->head.length  = length;
    
    spi_transfer_8bit(WIFI_SPI_INDEX, (uint8 *)&(read_data->head), (uint8 *)&(read_data->head), sizeof(wifi_spi_head_struct));
    
    if(WIFI_SPI_RECVIVE_SIZE < length)
    {
        spi_transfer_8bit(WIFI_SPI_INDEX, write_data, read_data->buffer, WIFI_SPI_RECVIVE_SIZE);
        spi_write_8bit_array(WIFI_SPI_INDEX, &write_data[WIFI_SPI_RECVIVE_SIZE], length - WIFI_SPI_RECVIVE_SIZE);
    }
    else
    {
        memcpy(read_data->buffer, write_data, length);
        spi_transfer_8bit(WIFI_SPI_INDEX, read_data->buffer, read_data->buffer, WIFI_SPI_RECVIVE_SIZE);
    }
    gpio_high(WIFI_SPI_CS_PIN);
}

static uint8 wifi_spi_set_parameter (wifi_spi_packets_command_enum command, uint8 *buffer, uint16 length, uint32 wait_time)
{
    uint8 return_state;
    wifi_spi_head_struct head;
    return_state = 1;
    do
    {
        head.command = command;
        head.length  = length;
        
        if(wifi_spi_wait_idle(wait_time))
        {
            break;
        }

        wifi_spi_write(&head.command, sizeof(wifi_spi_head_struct), buffer, length);
        if(wifi_spi_wait_idle(wait_time))
        {
            break;
        }

        head.command = WIFI_SPI_DATA;
        head.length = 0;
        wifi_spi_transfer_command((wifi_spi_packets_struct *)&head, head.length);
        system_delay_us(20);
        if(WIFI_SPI_REPLY_OK == head.command)
        {
            return_state = 0;
        }
    }while(0);
    
    return return_state;
}

static uint8 wifi_spi_get_parameter (wifi_spi_packets_command_enum command, wifi_spi_packets_struct *read_data, uint32 wait_time)
{
    uint8 return_state;

    return_state = 1;
    do
    {
        if(wifi_spi_wait_idle(wait_time))
        {
            break;
        }
        read_data->head.command = command;
        wifi_spi_write(&(read_data->head.command), WIFI_SPI_RECVIVE_SIZE, NULL, 0);

        if(wifi_spi_wait_idle(wait_time))
        {
            break;
        }
        read_data->head.command = WIFI_SPI_DATA;
        read_data->head.length = 0;
        wifi_spi_transfer_command(read_data, WIFI_SPI_RECVIVE_SIZE);
        return_state = 0;
    }while(0);
    return return_state;
}

static uint8 wifi_spi_get_version (void)
{
    uint8 return_state;
    wifi_spi_packets_struct temp_packets;

    return_state = wifi_spi_get_parameter(WIFI_SPI_GET_VERSION, &temp_packets, OTHER_TIME_OUT);
    if((0 == return_state) && (WIFI_SPI_REPLY_VERSION == temp_packets.head.command))
    {
        memcpy(wifi_spi_version, temp_packets.buffer, temp_packets.head.length);
    }
    return_state = (return_state == 0) ? (WIFI_SPI_REPLY_VERSION != temp_packets.head.command) : 1;

    return return_state;
}

static uint8 wifi_spi_get_mac_addr (void)
{
    uint8 return_state;
    wifi_spi_packets_struct temp_packets;

    return_state = wifi_spi_get_parameter(WIFI_SPI_GET_MAC_ADDR, &temp_packets, OTHER_TIME_OUT);
    if((0 == return_state) && (WIFI_SPI_REPLY_MAC_ADDR == temp_packets.head.command))
    {
        memcpy(wifi_spi_mac_addr, temp_packets.buffer, temp_packets.head.length);
    }
    return_state = (return_state == 0) ? (WIFI_SPI_REPLY_MAC_ADDR != temp_packets.head.command) : 1;

    return return_state;
}

static uint8 wifi_spi_get_ip_addr_port (void)
{
    uint8 return_state;
    wifi_spi_packets_struct temp_packets;

    return_state = wifi_spi_get_parameter(WIFI_SPI_GET_IP_ADDR, &temp_packets, OTHER_TIME_OUT);
    if((0 == return_state) && (WIFI_SPI_REPLY_IP_ADDR == temp_packets.head.command))
    {
        memcpy(wifi_spi_ip_addr_port, temp_packets.buffer, temp_packets.head.length);
    }
    return_state = (return_state == 0) ? (WIFI_SPI_REPLY_IP_ADDR != temp_packets.head.command) : 1;

    return return_state;
}

uint8 wifi_spi_wifi_connect (char *wifi_ssid, char *pass_word)
{
    uint8 return_state;
    uint8 temp_buffer[64];
    uint16 length;
    
    if(NULL != pass_word)
    {
        length = (uint16)sprintf((char *)temp_buffer, "%s\r\n%s\r\n", wifi_ssid, pass_word);
    }
    else
    {
        length = (uint16)sprintf((char *)temp_buffer, "%s\r\n", wifi_ssid);
    }

    return_state = wifi_spi_set_parameter(WIFI_SPI_SET_WIFI_INFORMATION, temp_buffer, length, WIFI_CONNECT_TIME_OUT);
    wifi_spi_get_ip_addr_port();

    return return_state;
}

uint8 wifi_spi_socket_connect (char *transport_type, char *ip_addr, char *port, char *local_port)
{
    uint8 return_state;
    uint8 temp_buffer[41];
    uint16 length;
    
    length = (uint16)sprintf((char *)temp_buffer, "%s\r\n%s\r\n%s\r\n%s\r\n", transport_type, ip_addr, port, local_port);

    return_state = wifi_spi_set_parameter(WIFI_SPI_SET_SOCKET_INFORMATION, temp_buffer, length, SOCKET_CONNECT_TIME_OUT);
    wifi_spi_get_ip_addr_port();

    return return_state;
}

uint8 wifi_spi_socket_disconnect (void)
{
    wifi_spi_packets_struct temp_packets;

    return wifi_spi_get_parameter(WIFI_SPI_CLOSE_SOCKET, &temp_packets, OTHER_TIME_OUT);
}

uint8 wifi_spi_reset (void)
{
    uint8 return_state;
    wifi_spi_head_struct head;
    return_state = 1;
    do
    {
        head.command = WIFI_SPI_RESET;
        head.length  = 0xA5A5;
        return_state = wifi_spi_wait_idle(OTHER_TIME_OUT);
        if(return_state)
        {
            break;
        }
        wifi_spi_write(&head.command, sizeof(wifi_spi_head_struct), NULL, 0);
    }while(0);
    
    return return_state;
}

uint8 wifi_spi_udp_send_now (void)
{
    uint8 return_state = 1;
    wifi_spi_packets_struct temp_packets;
    
    if(WIFI_SPI_IDLE == wifi_spi_mutex)
    {
        wifi_spi_mutex = WIFI_SPI_BUSY;
        do
        {
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }

            temp_packets.head.command = WIFI_SPI_UDP_SEND;
            temp_packets.head.length = 0;
            wifi_spi_transfer_command(&temp_packets, WIFI_SPI_RECVIVE_SIZE);
            
            if((WIFI_SPI_REPLY_DATA_START == temp_packets.head.command) || (WIFI_SPI_REPLY_DATA_END == temp_packets.head.command))
            {
                if(temp_packets.head.length)
                {
                    fifo_write_buffer(&wifi_spi_fifo, temp_packets.buffer, temp_packets.head.length);
                }
            }
            
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }
            
            temp_packets.head.command = WIFI_SPI_DATA;
            temp_packets.head.length = 0;
            wifi_spi_transfer_command(&temp_packets, temp_packets.head.length);
            
            if(WIFI_SPI_REPLY_OK == temp_packets.head.command)
            {
                return_state = 0;
            }
            
        }while(0);
        
        wifi_spi_mutex = WIFI_SPI_IDLE;
    } 
    
    return return_state;
}

uint32 wifi_spi_send_buffer (const uint8 *buffer, uint32 length)
{
    uint16 send_length;
    wifi_spi_packets_struct temp_packets;
    
    if(WIFI_SPI_IDLE == wifi_spi_mutex)
    {
        wifi_spi_mutex = WIFI_SPI_BUSY;
        
        while(length)
        {
            send_length = length > WIFI_SPI_TRANSFER_SIZE ? (uint16)WIFI_SPI_TRANSFER_SIZE : (uint16)length;
            
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }
            
            wifi_spi_transfer_data(buffer, &temp_packets, send_length);
            
            if((WIFI_SPI_REPLY_DATA_START == temp_packets.head.command) || (WIFI_SPI_REPLY_DATA_END == temp_packets.head.command))
            {
                if(temp_packets.head.length)
                {
                    fifo_write_buffer(&wifi_spi_fifo, temp_packets.buffer, temp_packets.head.length);
                }
            }
            
            length -= send_length;
            buffer += send_length;
        }
        
        while(WIFI_SPI_REPLY_DATA_START == temp_packets.head.command)
        {
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }
            
            temp_packets.head.command = WIFI_SPI_DATA;
            temp_packets.head.length  = 0;
            wifi_spi_transfer_command(&temp_packets, WIFI_SPI_RECVIVE_SIZE);
            if((WIFI_SPI_REPLY_DATA_START == temp_packets.head.command) || (WIFI_SPI_REPLY_DATA_END == temp_packets.head.command))
            {
                if(temp_packets.head.length)
                {
                    fifo_write_buffer(&wifi_spi_fifo, temp_packets.buffer, temp_packets.head.length);
                }
            }
        }
        wifi_spi_mutex = WIFI_SPI_IDLE;
    }
    return length;
}

uint32 wifi_spi_read_buffer (uint8 *buffer, uint32 length)
{
    zf_assert(NULL != buffer);
    uint32 data_len = length;
    
#if(1 == WIFI_SPI_READ_TRANSFER)
    wifi_spi_packets_struct temp_packets;
    if(WIFI_SPI_IDLE == wifi_spi_mutex)
    {
        wifi_spi_mutex = WIFI_SPI_BUSY;
        do
        {
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }
            temp_packets.head.command = WIFI_SPI_DATA;
            temp_packets.head.length  = 0;
            wifi_spi_transfer_command(&temp_packets, WIFI_SPI_RECVIVE_SIZE);
            if((WIFI_SPI_REPLY_DATA_START == temp_packets.head.command) || (WIFI_SPI_REPLY_DATA_END == temp_packets.head.command))
            {
                if(temp_packets.head.length)
                {
                    fifo_write_buffer(&wifi_spi_fifo, temp_packets.buffer, temp_packets.head.length);
                }
            }
        }while(WIFI_SPI_REPLY_DATA_START == temp_packets.head.command);
        wifi_spi_mutex = WIFI_SPI_IDLE;
    }
#endif 
    
    fifo_read_buffer(&wifi_spi_fifo, buffer, &data_len, FIFO_READ_AND_CLEAN);
    return data_len;
}

uint8 wifi_spi_init (char *wifi_ssid, char *pass_word)
{
    uint8 return_state = 0;
    
    fifo_init(&wifi_spi_fifo, FIFO_DATA_8BIT, wifi_spi_buffer, WIFI_SPI_RECVIVE_FIFO_SIZE);
    spi_init(WIFI_SPI_INDEX, SPI_MODE0, WIFI_SPI_SPEED, WIFI_SPI_SCK_PIN, WIFI_SPI_MOSI_PIN, WIFI_SPI_MISO_PIN, SPI_CS_NULL);
    gpio_init(WIFI_SPI_CS_PIN,  GPO, 1, GPO_PUSH_PULL);
    gpio_init(WIFI_SPI_RST_PIN, GPO, 1, GPO_PUSH_PULL);
    gpio_init(WIFI_SPI_INT_PIN, GPI, 0, GPI_PULL_DOWN);
    
    gpio_set_level(WIFI_SPI_RST_PIN, 0);
    system_delay_ms(10);
    gpio_set_level(WIFI_SPI_RST_PIN, 1);
    
    system_delay_ms(100);
    wifi_spi_mutex = WIFI_SPI_IDLE;

    do
    {
        return_state = wifi_spi_get_version();
        if(return_state)
        {
            break;
        }

        wifi_spi_get_mac_addr();

        if(NULL == wifi_ssid)
        {
            break;
        }

        return_state = wifi_spi_wifi_connect(wifi_ssid, pass_word);
        if(return_state)
        {
            break;
        }
        
    #if(1 == WIFI_SPI_AUTO_CONNECT)
        return_state = wifi_spi_socket_connect("TCP", WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT);
        if(return_state)
        {
            break;
        }
    #endif
        
    #if(2 == WIFI_SPI_AUTO_CONNECT)
        return_state = wifi_spi_socket_connect("UDP", WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT);
        if(return_state)
        {
            break;
        }
    #endif
    }while(0);

    return return_state;
}
