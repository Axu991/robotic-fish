/**
 * @file main.c
 * @brief BRIZO 固件系统入口
 */

#include <stdint.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "app/robot_app.h"
#include "app/cpg_gait.h"

#include "drivers/adc_driver.h"
#include "drivers/can_servo_driver.h"
#include "drivers/crsf_driver.h"
#include "drivers/imu901_driver.h"
#include "drivers/led_driver.h"
#include "drivers/udp_driver.h"

static const char *TAG = "MAIN";

#define PIN_CAN_TX      GPIO_NUM_4
#define PIN_CAN_RX      GPIO_NUM_5

#define PIN_IMU_TX      GPIO_NUM_6
#define PIN_IMU_RX      GPIO_NUM_7
#define IMU_UART_PORT   UART_NUM_1

#define PIN_CRSF_RX     GPIO_NUM_2
#define PIN_CRSF_TX     GPIO_NUM_37
#define CRSF_UART_PORT  UART_NUM_2

#define PIN_ADC_TEMP    GPIO_NUM_16
#define PIN_LED_STATUS  GPIO_NUM_17

#define UDP_LOCAL_PORT  2333
#define UDP_REMOTE_PORT 6060

#define IPV4_HOST(a, b, c, d) \
    ((((uint32_t)(a)) << 24) | (((uint32_t)(b)) << 16) | \
     (((uint32_t)(c)) << 8) | ((uint32_t)(d)))

static void init_network_stack(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing BRIZO firmware");

    led_init(PIN_LED_STATUS);
    adc_init(PIN_ADC_TEMP);

    if (!can_servo_init(PIN_CAN_TX, PIN_CAN_RX)) {
        ESP_LOGE(TAG, "TWAI CAN servo initialization failed");
        return;
    }
    for (int i = 0; i < CPG_JOINTS_NUM; i++) {
        can_servo_set_position(0x25 + i, 0.0f);
    }

    crsf_init(CRSF_UART_PORT, PIN_CRSF_RX, PIN_CRSF_TX);
    imu_init(IMU_UART_PORT, PIN_IMU_TX, PIN_IMU_RX, 115200);

    init_network_stack();

    wifi_udp_info_t udp_configs[] = {
        {
            .ssid = "SHTechOxbridge",
            .password = "limad206",
            .local_port = UDP_LOCAL_PORT,
            .remote_port = UDP_REMOTE_PORT,
            .remote_ip = IPV4_HOST(192, 168, 50, 37),
        },
        {
            .ssid = "MAgIC_Fish_Project",
            .password = "123456789",
            .local_port = UDP_LOCAL_PORT,
            .remote_port = UDP_REMOTE_PORT,
            .remote_ip = IPV4_HOST(192, 168, 1, 105),
        },
        {
            .ssid = "iPhone",
            .password = "66668888",
            .local_port = UDP_LOCAL_PORT,
            .remote_port = UDP_REMOTE_PORT,
            .remote_ip = IPV4_HOST(172, 20, 10, 1),
        },
    };
    if (!udp_driver_init(udp_configs,
                         sizeof(udp_configs) / sizeof(udp_configs[0]))) {
        ESP_LOGW(TAG, "WiFi/UDP unavailable; local control remains enabled");
    }

    if (!robot_app_start()) {
        ESP_LOGE(TAG, "Robot application startup failed");
        return;
    }

    ESP_LOGI(TAG, "BRIZO firmware initialized");
}
