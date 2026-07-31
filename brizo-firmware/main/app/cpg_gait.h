/**
 * @file cpg_gait.h
 * @brief 机器鱼 CPG 步态生成算法模块 (ESP-IDF)
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-31
 * @version 1.0
 */

#ifndef CPG_GAIT_H
#define CPG_GAIT_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG2RAD(deg) ((deg) * (M_PI / 180.0f))
#define RAD2DEG(rad) ((rad) * (180.0f / M_PI))

#define CPG_JOINTS_NUM    3
#define REPLAY_FRAME_NUMS 50

typedef struct {
    float dt;                      /*!< 步长周期 dt (s)，默认 0.02s (50Hz) */
    float f;                       /*!< 当前频率 (Hz) */
    
    /* 目标参数 [rad] */
    float R[CPG_JOINTS_NUM];       /*!< 目标振幅 */
    float X[CPG_JOINTS_NUM];       /*!< 目标偏置 */
    float P[CPG_JOINTS_NUM][CPG_JOINTS_NUM]; /*!< 目标相位差矩阵 */

    /* 微分方程中间状态量 */
    float r[CPG_JOINTS_NUM];
    float x[CPG_JOINTS_NUM];
    float p[CPG_JOINTS_NUM];

    float r_[CPG_JOINTS_NUM];
    float x_[CPG_JOINTS_NUM];
    float p_[CPG_JOINTS_NUM];

    /* 收敛增益参数 */
    float C_r;
    float C_x;
    float C_p;

    /* 重放机动控制状态 */
    bool replay_active;
    int replay_frame_index;
    int replay_direction;          /*!< +1: 右播放, -1: 左播放 */
} cpg_handle_t;

void cpg_gait_init(cpg_handle_t *cpg, float dt_s);

/**
 * @brief 更新一次 CPG 积分步长，输出 3 关节角度 (rad)
 */
void cpg_gait_update(cpg_handle_t *cpg, float out_joint_angles_rad[CPG_JOINTS_NUM]);

/**
 * @brief 触发动作重放机动
 * @param direction +1 为右播放，-1 为左播放
 */
void cpg_gait_trigger_replay(cpg_handle_t *cpg, int direction);

#ifdef __cplusplus
}
#endif

#endif // CPG_GAIT_H