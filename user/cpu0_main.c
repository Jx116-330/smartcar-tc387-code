#include "myhead.h"
#pragma section all "cpu0_dsram"

#define WIFI_SSID_TEST          "Jx116"
#define WIFI_PASSWORD_TEST      "777888999"
#define UDP_TARGET_IP           "172.20.10.3"
#define UDP_TARGET_PORT         "8086"
#define WIFI_LOCAL_PORT         "6666"

uint8 wifi_spi_test_buffer[] = "this is wifi spi test buffer";
uint8 wifi_spi_get_data_buffer[256];
uint32 data_length;

int core0_main(void)
{
    clock_init();
    debug_init();

    while (wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);
    }

    printf("\r\n module version:%s", wifi_spi_version);
    printf("\r\n module mac    :%s", wifi_spi_mac_addr);
    printf("\r\n module ip     :%s", wifi_spi_ip_addr_port);

    if (0 == WIFI_SPI_AUTO_CONNECT)
    {
        while (wifi_spi_socket_connect("UDP", UDP_TARGET_IP, UDP_TARGET_PORT, WIFI_LOCAL_PORT))
        {
            printf("\r\n Connect UDP Servers error, try again.");
            system_delay_ms(100);
        }
    }

    data_length = wifi_spi_send_buffer(wifi_spi_test_buffer, sizeof(wifi_spi_test_buffer));
    if (!data_length)
    {
        printf("\r\n send success.");
    }
    else
    {
        printf("\r\n %ld bytes data send failed.", data_length);
    }

    cpu_wait_event_ready();
    while (TRUE)
    {
        data_length = wifi_spi_read_buffer(wifi_spi_get_data_buffer, sizeof(wifi_spi_get_data_buffer));
        if (data_length)
        {
            printf("\r\n Get data: <%s>.", wifi_spi_get_data_buffer);
            if (!wifi_spi_send_buffer(wifi_spi_get_data_buffer, data_length))
            {
                wifi_spi_udp_send_now();
                printf("\r\n send success.");
                memset(wifi_spi_get_data_buffer, 0, data_length);
            }
            else
            {
                printf("\r\n %ld bytes data send failed.", data_length);
            }
        }
        system_delay_ms(100);
    }
}

#pragma section all restore
