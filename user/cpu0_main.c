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
 
    while (TRUE)
    {
        gnss_data_parse();

        if (path_recorder_get_state() == PATH_STATE_RECORDING)
        {
            path_recorder_task();
        }
       menu_task();
      
    }

    
}
#pragma section all restore
