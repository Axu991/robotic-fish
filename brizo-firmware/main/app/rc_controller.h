/**
 * @file rc_controller.h
 * @brief 机器鱼遥控器手柄映射与模式状态机模块 (ESP-IDF)
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-31
 * @version 1.0
 */

#ifndef RC_CONTROLLER_H
#define RC_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "cpg_gait.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CONTROL_SRC_INSIDE_CPG = 0,  /*!< 内部 CPG / AHC / 预设重放控制 */
    CONTROL_SRC_DOWNWARD_UDP     /*!< 上位机 UDP 命令直控 */
} control_source_e;

typedef struct {
    uint16_t crsf_ch[10];        /*!< 10 通道 raw CRSF 脉宽值 (1000~2000us) */
    float current_yaw_deg;       /*!< IMU 实时航向角 (用于 AHC 闭环) */
} rc_input_data_t;

void rc_controller_init(void);

/**
 * @brief 处理遥控器输入，直接更新 CPG 句柄的目标参数并返回控制源模式
 */
control_source_e rc_controller_update(const rc_input_data_t *input, cpg_handle_t *cpg);

#ifdef __cplusplus
}
#endif

#endif // RC_CONTROLLER_H