/**
 * @file cpg_test_task.c
 * @brief BRIZO-KM CPG 步态生成模块测试程序 (ESP-IDF)
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-31
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "cpg_gait.h"

static const char *TAG = "CPG_TEST";

/**
 * @brief 在终端打印简易 ASCII 波形图 (方便视觉化观察 3 关节相对相位)
 */
static void print_ascii_plot(float theta1, float theta2, float theta3)
{
    const int width = 30; // 打印宽度
    const float max_angle = 0.5f; // 最大映射角度范围 (~28度)

    int p1 = (int)((theta1 / max_angle) * (width / 2)) + (width / 2);
    int p2 = (int)((theta2 / max_angle) * (width / 2)) + (width / 2);
    int p3 = (int)((theta3 / max_angle) * (width / 2)) + (width / 2);

    char line[width + 1];
    for (int i = 0; i < width; i++) line[i] = ' ';
    line[width] = '\0';
    line[width / 2] = '|'; // 中线 (0 rad)

    if (p1 >= 0 && p1 < width) line[p1] = '1';
    if (p2 >= 0 && p2 < width) line[p2] = '2';
    if (p3 >= 0 && p3 < width) line[p3] = '3';

    printf("[%s]\n", line);
}

void cpg_test_task(void *pvParameters)
{
    cpg_handle_t cpg;
    float joint_angles[CPG_JOINTS_NUM] = {0.0f};

    // 1. 初始化 CPG (100Hz 步长，dt = 0.01s)
    cpg_gait_init(&cpg, 0.01f);

    // 2. 配置 CPG 参数
    // 摆幅: 0.35 rad (~20度), 摆频: 1.0 Hz, 关节相位差: 45度 (0.785 rad)
    float offsets[3] = {0.0f, 0.0f, 0.0f}; // 零偏置直游
    cpg_gait_set_params(&cpg, 0.35f, 1.0f, 0.785f, offsets);

    ESP_LOGI(TAG, "=== CPG 步态生成器测试开始 ===");
    ESP_LOGI(TAG, "配置: 幅值=0.35rad, 频率=1.0Hz, 相位差=45deg");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t step_count = 0;

    while (1) {
        // 100Hz 执行 CPG 数值积分解算
        cpg_gait_update(&cpg, joint_angles);

        // 每 10 次解算（即 10Hz/100ms）在终端打印一次数值与波形
        if (step_count % 10 == 0) {
            float deg1 = joint_angles[0] * 57.29578f; // rad 转换为 deg
            float deg2 = joint_angles[1] * 57.29578f;
            float deg3 = joint_angles[2] * 57.29578f;

            // 1. 打印精准数值 (弧度与角度)
            printf("[CPG Output] T1: %6.3f rad (%6.1f deg) | T2: %6.3f rad (%6.1f deg) | T3: %6.3f rad (%6.1f deg)  ",
                   joint_angles[0], deg1,
                   joint_angles[1], deg2,
                   joint_angles[2], deg3);

            // 2. 打印对应波形图 (1, 2, 3 分别代表三关节位置)
            print_ascii_plot(joint_angles[0], joint_angles[1], joint_angles[2]);
        }

        // 动态测试：运行 10 秒后自动注入转向偏置 (Bias Offset)
        if (step_count == 1000) {
            ESP_LOGW(TAG, ">>> 模拟 AHC 注入右转偏置 offset = [0.10, 0.05, 0.00] <<<");
            float turn_offsets[3] = {0.10f, 0.05f, 0.00f};
            cpg_gait_set_params(&cpg, 0.35f, 1.0f, 0.785f, turn_offsets);
        }

        step_count++;
        // 严格维持 10ms (100Hz) 控制周期
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

// app_main 入口测试点
void app_main(void)
{
    // 创建 CPG 测试任务 (分配 4096 字节栈空间)
    xTaskCreate(cpg_test_task, "cpg_test_task", 4096, NULL, 5, NULL);
}