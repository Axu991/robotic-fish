/**
 * @file crsf_driver.h
 * @brief CRSF 协议驱动头文件
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-31
 * @version 1.0
 */

#ifndef CRSF_DRIVER_H
#define CRSF_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CRSF_CHANNEL_COUNT  10      // 通道数
#define CRSF_BAUDRATE       420000  // CRSF 标准波特率

/**
 * @brief CRSF 通道数据结构
 */
typedef struct {
    uint16_t ch[CRSF_CHANNEL_COUNT]; // 通道值范围约 988 ~ 2012
} crsf_channels_t;

/**
 * @brief 初始化 CRSF 接收
 * @param uart_num UART 端口号（如 UART_NUM_2）
 * @param rx_pin   GPIO 接收引脚（连接 CRSF 接收机的 TX）
 * @param tx_pin   GPIO 发送引脚（通常不用，可设 UART_PIN_NO_CHANGE）
 */
void crsf_init(uart_port_t uart_num, gpio_num_t rx_pin, gpio_num_t tx_pin);

/**
 * @brief 获取最新 CRSF 通道数据（非阻塞）
 * @param[out] channels 填充通道数据
 * @return true 成功获取（至少有一帧有效数据），false 无新数据
 */
bool crsf_get_channels(crsf_channels_t *channels);

/**
 * @brief 获取单个通道值（便捷函数）
 * @param index 通道索引（0~9）
 * @return 通道值（若未初始化则返回 1500）
 */
uint16_t crsf_get_channel(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif