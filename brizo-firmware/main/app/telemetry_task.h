/**
 * @file telemetry_task.h
 * @brief 机器鱼遥测与控制主任务模块 (ESP-IDF)
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-31
 * @version 1.0
 */

#ifndef TELEMETRY_TASK_H
#define TELEMETRY_TASK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化应用层全局任务 (创建 50Hz 遥测任务与 100Hz 控制主循环)
 * 
 * @return true 启动成功, false 失败
 */
bool telemetry_task_start(void);

#ifdef __cplusplus
}
#endif

#endif // TELEMETRY_TASK_H