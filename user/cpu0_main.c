#include "myhead.h"
#pragma section all "cpu0_dsram"

#define WIFI_SSID_TEST          "Jx116"
#define WIFI_PASSWORD_TEST      "12345678"
#define TCP_TARGET_IP           "172.20.10.3"
#define TCP_TARGET_PORT         "8086"                      // 连接目标的端口
#define WIFI_LOCAL_PORT         "6666"                      // 本机的端口 0：随机  可设置范围2048-65535  默认 6666

uint8 wifi_spi_test_buffer[] = "this is wifi spi test buffer";
uint8 wifi_spi_get_data_buffer[256];
uint32 data_length;

int core0_main(void)
{
    clock_init();
    debug_init();
    Beep_Init();
    gnss_init(TAU1201);
    path_recorder_init();
    menu_init();
    cpu_wait_event_ready();
/** 
    while (TRUE)
    {
        gnss_data_parse();

        if (path_recorder_get_state() == PATH_STATE_RECORDING)
        {
            path_recorder_task();
        }
       menu_task();
      
    }
*/
    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);                                                   // 初始化失败 等待 100ms
    }

    printf("\r\n module version:%s",wifi_spi_version);                          // 模块固件版本
    printf("\r\n module mac    :%s",wifi_spi_mac_addr);                         // 模块 MAC 信息
    printf("\r\n module ip     :%s",wifi_spi_ip_addr_port);                     // 模块 IP 地址

    // zf_device_wifi_spi.h 文件内的宏定义可以更改模块连接(建立) WIFI 之后，是否自动连接 TCP 服务器、创建 UDP 连接
    if(0 == WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开启自动连接 就需要手动连接目标 IP
    {
        while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
            "TCP",                                                              // 指定使用TCP方式通讯
            TCP_TARGET_IP,                                                      // 指定远端的IP地址，填写上位机的IP地址
            TCP_TARGET_PORT,                                                    // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
            WIFI_LOCAL_PORT))                                                   // 指定本机的端口号
        {
            // 如果一直建立失败 考虑一下是不是没有接硬件复位
            printf("\r\n Connect TCP Servers error, try again.");
            system_delay_ms(100);                                               // 建立连接失败 等待 100ms
        }
    }


    // 发送测试数据至服务器
    data_length = wifi_spi_send_buffer(wifi_spi_test_buffer, sizeof(wifi_spi_test_buffer));
    if(!data_length)
    {
        printf("\r\n send success.");
    }
    else
    {
        printf("\r\n %ld bytes data send failed.", data_length);
    }

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    while (TRUE)
    {
        // 此处编写需要循环执行的代码
        data_length = wifi_spi_read_buffer(wifi_spi_get_data_buffer, sizeof(wifi_spi_get_data_buffer));
        if(data_length)                                                     // 如果接收到数据 则进行数据类型判断
        {
            printf("\r\n Get data: <%s>.", wifi_spi_get_data_buffer);
            if(!wifi_spi_send_buffer(wifi_spi_get_data_buffer, data_length))
            {
                printf("\r\n send success.");
                memset(wifi_spi_get_data_buffer, 0, data_length);           // 数据发送完成 清空数据
            }
            else
            {
                printf("\r\n %ld bytes data send failed.", data_length);
            }
        }
        system_delay_ms(100);
        // 此处编写需要循环执行的代码
    }
}
#pragma section all restore
