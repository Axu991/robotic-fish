/**
 * @file robot_app.h
 * @brief 机器鱼应用任务入口
 */

#ifndef ROBOT_APP_H
#define ROBOT_APP_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化应用状态并创建控制、传感器、遥测和 UDP 接收任务
 * @return true 全部任务创建成功，false 初始化失败
 */
bool robot_app_start(void);

#ifdef __cplusplus
}
#endif

#endif // ROBOT_APP_H
