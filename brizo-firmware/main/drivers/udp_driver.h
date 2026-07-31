/** 
 * @file udp_driver.h
 * @brief UDP 驱动头文件
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-31
 * @version 1.3
 */

#ifndef UDP_DRIVER_H
#define UDP_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char* ssid;
    const char* password;
    uint16_t local_port;
    uint16_t remote_port;
    uint32_t remote_ip;   // 主机序。若填 0，则自动使用动态获取到的 Gateway IP
} wifi_udp_info_t;

/**
 * @brief 初始化 WiFi 并建立 UDP Socket
 * @param configs 配置数组指针
 * @param count 配置项数量
 * @return true 初始化成功, false 失败
 */
bool udp_driver_init(wifi_udp_info_t *configs, int count);

/**
 * @brief 发送 UDP 数据包
 */
int udp_send_packet(const uint8_t *data, size_t len);

/**
 * @brief 接收 UDP 数据包 (带 100ms 套接字接收超时)
 */
int udp_receive_packet(uint8_t *buffer, size_t max_len);

/**
 * @brief 检查 UDP 是否可通信
 */
bool udp_is_connected(void);

/**
 * @brief 获取当前获取到的网关 IP (主机序)
 */
uint32_t udp_get_gateway_ip(void);

#ifdef __cplusplus
}
#endif

#endif // UDP_DRIVER_H