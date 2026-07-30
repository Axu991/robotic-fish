/** 
 * @file can_servo_driver.h
 * @brief Header file for the CAN servo driver.
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-30
 * @version 1.0
*/

#ifndef CAN_SERVO_DRIVER_H
#define CAN_SERVO_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

ifdef __cplusplus
extern "C" {
#endif

/** @brief 舵机反馈数据结构
 */
typedef struct {
    uint8_t id;          /**< 舵机ID */
    float position;     /**< 舵机位置 */
    float current;      /**< 舵机电流 */
    int16_t temperature;  /**< 舵机温度 */
}servo_feedback_t;

/**
 *  @brief 初始化 TWAI 舵机驱动
 *  @param tx_pin GPIO 输出引脚
 *  @param rx_pin GPIO 输入引脚
 *  @return true 初始化成功，false 初始化失败
 */
bool can_servo_init(gpio_num_t tx_pin, gpio_num_t rx_pin);

/**
 *  @brief 发送舵机控制命令（阻塞发生）
 *  @param node_id 舵机节点ID（如0x25)
 *  @param position_deg 目标位置（单位：度）
 *  @return true 发送成功，false 发送失败
 */
bool can_servo_set_position(uint8_t node_id, float position_deg);

/**
 * @brief 读取舵机当前位置（阻塞读取，带超时）
 * @param node_id 舵机节点ID（如0x25)
 * @param[out] position 当前位置（单位：度）
 * @return true 读取成功，false 读取失败
 */
bool can_servo_get_position(uint8_t node_id, float *position);

/**
 * @brief 读取舵机电流（阻塞读取，带超时）
 * @param node_id 舵机节点ID（如0x25)
 * @param[out] current 舵机电流（单位：A）
 * @param[out] temperature 舵机温度（单位：摄氏度）
 * @return true 读取成功，false 读取失败
 */
bool can_servo_get_current(uint8_t node_id, float *current, int16_t *temperature);

/**
 * @brief 一次性读取所有反馈数据（非阻塞，用于循环调用）
 * @param fb 指向 servo_feedback_t 结构体的指针
 * @param count 需要读取舵机的数量
 * @return 读取成功的舵机数量
 */
int can_servo_read_all_feedback(servo_feedback_t *fb, int count);

#ifdef __cplusplus
}#endif

#endif // CAN_SERVO_DRIVER_H