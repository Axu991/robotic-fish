/**
 * @file udp_driver.c
 * @brief UDP 驱动实现文件
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-31
 * @version 1.3
 */

#include "udp_driver.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <string.h>
#include <inttypes.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>

static const char *TAG = "UDP_DRIVER";

static int s_sock = -1;
static struct sockaddr_in s_remote_addr;
static bool s_connected = false;
static bool s_wifi_initialized = false;

// 事件组同步机制
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;
#define MAXIMUM_RETRY  5

// 动态获取的 Gateway IP (主机序)
static uint32_t s_gateway_ip_host_order = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP (%d/%d)", s_retry_num, MAXIMUM_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got Local IP:" IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Got Gateway IP:" IPSTR, IP2STR(&event->ip_info.gw));

        // 提取 Gateway IP (转换为 Host 字节序保存)
        s_gateway_ip_host_order = ntohl(event->ip_info.gw.addr);

        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static bool wifi_connect(const char *ssid, const char *password)
{
    s_retry_num = 0;

    if (!s_wifi_event_group) {
        s_wifi_event_group = xEventGroupCreate();
    }

    if (!s_wifi_initialized) {
        esp_netif_create_default_wifi_sta();
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        // 注册事件回调
        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        s_wifi_initialized = true;
    }

    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 等待连接成功获取 IP 或超过最大重试次数失败
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdTRUE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Successfully connected to AP: %s", ssid);
        return true;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to AP: %s", ssid);
        esp_wifi_stop();
        return false;
    }

    return false;
}

static bool create_udp_socket(uint16_t local_port, uint32_t remote_ip, uint16_t remote_port)
{
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "Socket creation failed");
        return false;
    }

    // 设置接收超时 100ms，让出 CPU 给 FreeRTOS
    struct timeval timeout = {
        .tv_sec = 0,
        .tv_usec = 100000 // 100ms
    };
    setsockopt(s_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    struct sockaddr_in local_addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port = htons(local_port)
    };
    if (bind(s_sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "Socket bind failed");
        close(s_sock);
        s_sock = -1;
        return false;
    }

    s_remote_addr.sin_family = AF_INET;
    s_remote_addr.sin_addr.s_addr = htonl(remote_ip);
    s_remote_addr.sin_port = htons(remote_port);

    s_connected = true;
    ESP_LOGI(TAG, "UDP socket bound to port %u, remote IP: %" PRIu32 ".%" PRIu32 ".%" PRIu32 ".%" PRIu32 ":%u",
             local_port,
             (remote_ip >> 24) & 0xFF, (remote_ip >> 16) & 0xFF,
             (remote_ip >> 8) & 0xFF, remote_ip & 0xFF, remote_port);
    return true;
}

bool udp_driver_init(wifi_udp_info_t *configs, int count)
{
    if (configs == NULL || count == 0) {
        ESP_LOGE(TAG, "No WiFi config provided");
        return false;
    }

    for (int i = 0; i < count; i++) {
        ESP_LOGI(TAG, "Attempting to connect to SSID: %s", configs[i].ssid);
        if (wifi_connect(configs[i].ssid, configs[i].password)) {
            
            uint32_t target_remote_ip = configs[i].remote_ip;
            // 如果设置为 0，则自动使用获取到的 Gateway IP
            if (target_remote_ip == 0) {
                target_remote_ip = s_gateway_ip_host_order;
                ESP_LOGI(TAG, "remote_ip is 0, auto-using Gateway IP");
            }

            if (create_udp_socket(configs[i].local_port,
                                  target_remote_ip,
                                  configs[i].remote_port)) {
                return true;
            }
        }
    }
    ESP_LOGE(TAG, "Failed to connect to any provided WiFi network");
    return false;
}

int udp_send_packet(const uint8_t *data, size_t len)
{
    if (!s_connected || s_sock < 0) {
        ESP_LOGE(TAG, "Socket not ready");
        return -1;
    }
    int sent = sendto(s_sock, data, len, 0,
                      (struct sockaddr *)&s_remote_addr, sizeof(s_remote_addr));
    if (sent < 0) {
        ESP_LOGE(TAG, "sendto failed, errno=%d", errno);
        return -1;
    }
    return sent;
}

int udp_receive_packet(uint8_t *buffer, size_t max_len)
{
    if (!s_connected || s_sock < 0) {
        return -1;
    }
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);
    int len = recvfrom(s_sock, buffer, max_len, 0,
                       (struct sockaddr *)&source_addr, &socklen);
    if (len < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0; // 超时无数据，正常返回 0
        }
        ESP_LOGE(TAG, "recvfrom failed, errno=%d", errno);
        return -1;
    }
    return len;
}

bool udp_is_connected(void)
{
    return s_connected && s_sock >= 0;
}

uint32_t udp_get_gateway_ip(void)
{
    return s_gateway_ip_host_order;
}