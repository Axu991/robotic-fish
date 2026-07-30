/**
 * @file    led_driver.h
 * @brief   LED 驱动接口（控制 LED 亮灭、闪烁、状态显示）
 * @author  huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date    2026-07-30
 * @version 1.0.0
 */

 #ifndef LED_DRIVER_H
#define LED_DRIVER_H

#include <stdbool.h>
#include "driver/gpio.h"

/**
 * @brief 初始化 LED 驱动
 * @param pin LED 所连接的 GPIO 引脚号
 */
void led_init(gpio_num_t pin);

/**
 * @brief 设置 LED 的状态
 * @param on true 表示点亮 LED，false 表示熄灭 LED
 */
void led_set(bool on);

/**
 * @brief 切换 LED 的状态
 */
void led_toggle(void);

/**
 * @brief 获取 LED 的状态
 * @return true 表示 LED 点亮，false 表示 LED 熄灭
 */
bool led_get_state(void);

#endif /* LED_DRIVER_H */