#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "protocols/udp_driver.h"

static const char *TAG = "MAIN";

// WiFi/UDP 配置列表
wifi_udp_info_t wifi_configs[] = {
    {
        .ssid = "iPhone",
        .password = "66668888",
        .local_port = 2333,
        .remote_port = 6060,
        .remote_ip = 0   // 填 0 会自动解析为网关 IP (iPhone 热点本机 IP)
    }
};

void app_main(void)
{
    // 1. 初始化 NVS Flash
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. 初始化底层网络堆栈与默认事件循环
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 3. 初始化 UDP 驱动
    int config_count = sizeof(wifi_configs) / sizeof(wifi_configs[0]);
    if (!udp_driver_init(wifi_configs, config_count)) {
        ESP_LOGE(TAG, "UDP driver initialization failed!");
        return;
    }
    ESP_LOGI(TAG, "UDP system ready!");

    // 4. 测试发送首个数据包
    const char *test_msg = "Hello from ESP32-S3!";
    udp_send_packet((const uint8_t *)test_msg, strlen(test_msg));

    // 5. 数据接收与轮询循环
    uint8_t rx_buffer[256];
    while (1) {
        int len = udp_receive_packet(rx_buffer, sizeof(rx_buffer) - 1);
        if (len > 0) {
            rx_buffer[len] = '\0';
            ESP_LOGI(TAG, "Received packet (%d bytes): %s", len, rx_buffer);
            
            // 收到数据后回传 ACK 响应
            const char *ack_msg = "ACK";
            udp_send_packet((const uint8_t *)ack_msg, strlen(ack_msg));
        }
        
        // 由于 udp_receive_packet 内部自带 100ms 超时让出 CPU，此处可以适当减小或保留任务延时
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}