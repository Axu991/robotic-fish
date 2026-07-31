/**
 * @file cpg_gait.c
 * @brief 机器鱼 CPG 步态生成算法实现
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-31
 * @version 1.0
 */

#include "cpg_gait.h"
#include <string.h>

/* 旧版固件中预设的 C 型机动/翻转轨迹 (50 帧，3 关节弧度) */
static const float s_replay_joint_data[REPLAY_FRAME_NUMS][3] = {
  {-0.03898f, -0.06082f, -0.03494f}, {-0.02620f, -0.05032f, -0.07481f},
  {-0.03122f, -0.05888f, -0.08976f}, {-0.10738f, -0.03606f, -0.08361f},
  {-0.20174f, -0.04272f, -0.05341f}, {-0.34872f, -0.00126f, -0.02837f},
  {-0.48927f, -0.04413f,  0.03390f}, {-0.63340f, -0.06761f,  0.05805f},
  {-0.71687f, -0.18855f,  0.10821f}, {-0.81217f, -0.28134f,  0.11366f},
  {-0.88173f, -0.41180f,  0.11711f}, {-0.96607f, -0.50746f,  0.04344f},
  {-0.99126f, -0.58499f, -0.09297f}, {-1.00891f, -0.63864f, -0.27453f},
  {-0.96671f, -0.70446f, -0.45983f}, {-0.90049f, -0.77842f, -0.60744f},
  {-0.79037f, -0.84701f, -0.75235f}, {-0.64397f, -0.87517f, -0.88990f},
  {-0.49502f, -0.86130f, -1.01655f}, {-0.36818f, -0.77597f, -1.11869f},
  {-0.27765f, -0.62724f, -1.20673f}, {-0.15285f, -0.50263f, -1.23870f},
  {-0.01394f, -0.39614f, -1.22644f}, { 0.12824f, -0.34201f, -1.15867f},
  { 0.30231f, -0.28912f, -1.04933f}, { 0.44101f, -0.23601f, -0.92063f},
  { 0.57634f, -0.10965f, -0.76071f}, { 0.63310f, -0.00644f, -0.61557f},
  { 0.65392f,  0.11310f, -0.45686f}, { 0.61189f,  0.20436f, -0.35499f},
  { 0.51793f,  0.31976f, -0.15989f}, { 0.38905f,  0.41603f,  0.03491f},
  { 0.26568f,  0.44075f,  0.29933f}, { 0.14678f,  0.45827f,  0.45599f},
  { 0.04296f,  0.44239f,  0.57397f}, {-0.06811f,  0.40305f,  0.64593f},
  {-0.14861f,  0.30927f,  0.71265f}, {-0.17192f,  0.16738f,  0.73218f},
  {-0.14922f,  0.02435f,  0.69028f}, {-0.10262f, -0.05477f,  0.52025f},
  {-0.02180f, -0.11396f,  0.36094f}, { 0.04255f, -0.13116f,  0.20463f},
  { 0.10945f, -0.14382f,  0.07045f}, { 0.12550f, -0.12091f, -0.05894f},
  { 0.14727f, -0.11015f, -0.14622f}, { 0.12804f, -0.10442f, -0.15543f},
  { 0.09120f, -0.06838f, -0.14394f}, { 0.03982f, -0.03229f, -0.14878f},
  { 0.02947f, -0.02846f, -0.13605f}, { 0.04921f, -0.07646f, -0.11398f}
};

void cpg_gait_init(cpg_handle_t *cpg, float dt_s)
{
    if (cpg == NULL) return;
    memset(cpg, 0, sizeof(cpg_handle_t));

    cpg->dt = (dt_s > 0.0f) ? dt_s : 0.02f;
    cpg->f = 1.0f;

    cpg->C_r = 11.68f;
    cpg->C_x = 14.40f;
    cpg->C_p = 5.84f;

    // 相位差耦合矩阵初始化
    cpg->P[0][1] = -0.698f;
    cpg->P[1][2] = -1.396f;
    cpg->P[1][0] = -cpg->P[0][1];
    cpg->P[2][1] = -cpg->P[1][2];
    cpg->P[0][2] = cpg->P[0][1] + cpg->P[1][2];
    cpg->P[2][0] = -cpg->P[0][2];

    cpg->replay_active = false;
    cpg->replay_frame_index = 0;
    cpg->replay_direction = 1;
}

void cpg_gait_trigger_replay(cpg_handle_t *cpg, int direction)
{
    if (cpg == NULL) return;
    cpg->replay_active = true;
    cpg->replay_direction = (direction >= 0) ? 1 : -1;
    cpg->replay_frame_index = 0;
}

void cpg_gait_update(cpg_handle_t *cpg, float out_joint_angles_rad[CPG_JOINTS_NUM])
{
    if (cpg == NULL || out_joint_angles_rad == NULL) return;

    // 1. 如果处于动作重放状态，直接按帧输出
    if (cpg->replay_active) {
        for (int i = 0; i < CPG_JOINTS_NUM; i++) {
            out_joint_angles_rad[i] = cpg->replay_direction * s_replay_joint_data[cpg->replay_frame_index][i];
        }

        cpg->replay_frame_index++;
        if (cpg->replay_frame_index >= REPLAY_FRAME_NUMS) {
            cpg->replay_frame_index = 0;
            cpg->replay_active = false;
        }
        return;
    }

    // 2. 正常 CPG 微分方程数值解算
    float p__[CPG_JOINTS_NUM] = {0.0f};

    // 状态积分更新
    for (int i = 0; i < CPG_JOINTS_NUM; i++) {
        cpg->r[i] += cpg->r_[i] * cpg->dt;
        cpg->x[i] += cpg->x_[i] * cpg->dt;
        cpg->p[i] += cpg->p_[i] * cpg->dt;
    }

    // 状态变化率更新
    for (int i = 0; i < CPG_JOINTS_NUM; i++) {
        cpg->r_[i] = cpg->C_r * (cpg->R[i] - cpg->r[i]);
        cpg->x_[i] = cpg->C_x * (cpg->X[i] - cpg->x[i]);
    }

    // 相位加速度 p__ 耦合解算
    for (int i = 0; i < CPG_JOINTS_NUM; i++) {
        p__[i] = 0.0f;
        for (int j = 0; j < CPG_JOINTS_NUM; j++) {
            if (i != j) {
                p__[i] += (cpg->C_p * (cpg->C_p * (cpg->p[j] - cpg->p[i] - cpg->P[i][j]) - 2.0f * (cpg->p_[i] - 2.0f * M_PI * cpg->f)));
            }
        }
        cpg->p_[i] += p__[i] * cpg->dt;
    }

    // 输出相位波形叠加
    for (int i = 0; i < CPG_JOINTS_NUM; i++) {
        out_joint_angles_rad[i] = cpg->x[i] + cpg->r[i] * sinf(cpg->p[i]);
    }
}