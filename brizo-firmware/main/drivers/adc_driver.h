/**
 * @file adc_driver.h
 * @brief ADC 驱动接口（模拟信号采集）
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-30
 * @version 1.0.0
 */

#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include <stdint.h>
#include "driver/gpio.h"

/**
 * @brief 初始化 ADC 驱动
 * @param pin ADC 所连接的 GPIO 引脚号
 */
void adc_init(gpio_num_t pin);

/**
 * @brief 读取 ADC 电压 的数值
 * @return 电压值（单位：毫伏）, 若读取失败则返回 -1
 */
int adc_read_millvoltage(void);

/**
 * @brief 读取 ADC 电压 的数值
 * @return 电压值（单位：伏特）, 若读取失败则返回 -1.0
 */
float adc_read_voltage(void);

#endif /* ADC_DRIVER_H */