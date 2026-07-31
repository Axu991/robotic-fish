/**
 * @file rc_controller.c
 * @brief 机器鱼遥控器手柄映射与状态机管理实现
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-31
 */


#include "rc_controller.h"
#include "esp_log.h"

static const char *TAG = "RC_CTRL";

/* 预设默认摆幅、摆频与偏置常数 */
static const float s_set_amp[3]  = {10.0f * DEG2RAD(1.0f), 20.0f * DEG2RAD(1.0f), 30.0f * DEG2RAD(1.0f)};
static const float s_set_bias[3] = {30.0f * DEG2RAD(1.0f), 30.0f * DEG2RAD(1.0f), 30.0f * DEG2RAD(1.0f)};
static const float s_set_freq    = 5.0f;

/* AHC 状态变量 */
static bool s_bootup_status = false;
static float s_desired_yaw_deg = 0.0f;

static inline float fclamp(float val, float O_Min, float O_Max) {
    if (val < O_Min) return O_Min;
    if (val > O_Max) return O_Max;
    return val;
}

static inline float fmap(float val, float I_Min, float I_Max, float O_Min, float O_Max) {
    return (((val - I_Min) * ((O_Max - O_Min) / (I_Max - I_Min))) + O_Min);
}

void rc_controller_init(void)
{
    s_bootup_status = false;
    s_desired_yaw_deg = 0.0f;
    ESP_LOGI(TAG, "RC Controller logic mapping initialized.");
}

control_source_e rc_controller_update(const rc_input_data_t *input, cpg_handle_t *cpg)
{
    if (input == NULL || cpg == NULL) return CONTROL_SRC_INSIDE_CPG;

    // 1. 判断 CH7 (crsf_values[6])：控制模式选择 (UDP 直控 vs 内部 CPG)
    control_source_e source = CONTROL_SRC_INSIDE_CPG;
    if (input->crsf_ch[6] > 1500) {
        source = CONTROL_SRC_DOWNWARD_UDP;
    } else {
        source = CONTROL_SRC_INSIDE_CPG;
    }

    // 2. 如果当前未处于动作重放状态，解析 CPG / AHC 与 重放触发
    if (!cpg->replay_active) {

        // --- (A) 解析 CH5 (crsf_values[4]) 姿态模式 ---
        if (input->crsf_ch[4] > 1750) {
            // [模式 1] 回中归零状态
            for (int i = 0; i < CPG_JOINTS_NUM; i++) {
                cpg->R[i] = 0.0f;
                cpg->X[i] = 0.0f;
                cpg->r[i] = 0.0f;
                cpg->x[i] = 0.0f;
                cpg->r_[i] = 0.0f;
                cpg->x_[i] = 0.0f;
            }
            s_bootup_status = false; // 退出 AHC
        } 
        else if (input->crsf_ch[4] <= 1750 && input->crsf_ch[4] > 1250) {
            // [模式 2] CPG 手动控制状态（无 AHC 主动航向控制）
            for (int i = 0; i < CPG_JOINTS_NUM; i++) {
                cpg->R[i] = s_set_amp[i] * fmap((float)input->crsf_ch[2], 1000.0f, 2000.0f, 1.0f, 0.0f);
                cpg->X[i] = s_set_bias[i] * fmap((float)input->crsf_ch[0], 1000.0f, 2000.0f, -1.0f, 1.0f);
            }
            cpg->f = s_set_freq * fmap((float)input->crsf_ch[1], 1000.0f, 2000.0f, 1.0f, 0.0f);
            s_bootup_status = false; // 退出 AHC
        } 
        else {
            // [模式 3] CPG 控制状态（带有 AHC 主动航向控制）
            if (!s_bootup_status) {
                s_desired_yaw_deg = input->current_yaw_deg;
                s_bootup_status = true;
            } else {
                // 摇杆控制目标航向积分增长
                s_desired_yaw_deg += 1.0f * 0.02f * fmap((float)input->crsf_ch[0], 1000.0f, 2000.0f, -1.0f, 1.0f);
                
                for (int i = 0; i < CPG_JOINTS_NUM; i++) {
                    cpg->R[i] = s_set_amp[i] * fmap((float)input->crsf_ch[2], 1000.0f, 2000.0f, 1.0f, 0.0f);
                    // 航向偏差反馈作为偏置 X_i
                    cpg->X[i] = 1.0f * (s_desired_yaw_deg - input->current_yaw_deg) * DEG2RAD(1.0f);
                }
                cpg->f = s_set_freq * fmap((float)input->crsf_ch[1], 1000.0f, 2000.0f, 1.0f, 0.0f);
            }
        }

        // --- (B) 检查重放触发开关 CH9 (crsf_values[8]) 和 CH10 (crsf_values[9]) ---
        if (input->crsf_ch[8] < 1250) {
            cpg_gait_trigger_replay(cpg, -1); // 左播放
        } else if (input->crsf_ch[9] < 1250) {
            cpg_gait_trigger_replay(cpg, 1);  // 右播放
        }
    }

    return source;
}
