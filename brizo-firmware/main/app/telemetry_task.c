/**
 * @file telemetry_task.c
 * @brief 机器鱼遥测与控制主任务实现
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-31
 */

#include "telemetry_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/* 引入 drivers */
#include "udp_driver.h"
#include "imu901_driver.h"
#include "can_servo_driver.h"
#include "crsf_driver.h"

/* 引入 protocols */
#include "upward_packer.h"
#include "downward_parser.h"

/* 引入 app 算法模块 */
#include "cpg_gait.h"
#include "rc_controller.h"

static const char *TAG = "APP_MAIN_TASK";

/* 系统全局 CPG 与控制句柄 */
static cpg_handle_t s_cpg_handle;
static downward_parser_t s_downward_parser;

/**
 * @brief 50Hz 高频遥测上报任务 (20ms 周期)
 */
static void telemetry_tx_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t tx_buffer[UPWARD_MAX_PACKET_LEN];
    size_t packed_len = 0;

    upward_payload_telemetry_t telemetry_data;

    ESP_LOGI(TAG, "Telemetry TX Task Started (50Hz / 20ms)...");

    while (1) {
        // 1. 从底层驱动中提取物理传感器数据
        // (1) 提取 CAN 舵机反馈
        for (int i = 0; i < SERVO_NUMS; i++) {
            telemetry_data.id[i] = 0x25 + i;
            telemetry_data.theta[i] = can_servo_get_angle_rad(i);
            telemetry_data.vin[i]   = can_servo_get_current_a(i);
            telemetry_data.temp[i]  = can_servo_get_temp_c(i);
        }

        // (2) 提取 IMU901 姿态数据
        imu901_data_t imu_raw;
        imu901_get_latest(&imu_raw);
        telemetry_data.rpy[0]  = imu_raw.roll;
        telemetry_data.rpy[1]  = imu_raw.pitch;
        telemetry_data.rpy[2]  = imu_raw.yaw;
        telemetry_data.acc[0]  = imu_raw.ax;
        telemetry_data.acc[1]  = imu_raw.ay;
        telemetry_data.acc[2]  = imu_raw.az;
        telemetry_data.gyro[0] = imu_raw.gx;
        telemetry_data.gyro[1] = imu_raw.gy;
        telemetry_data.gyro[2] = imu_raw.gz;

        // 2. 调用 upward_packer 打包为协议格式 (0xAA 0x55)
        if (upward_pack_telemetry(&telemetry_data, tx_buffer, &packed_len)) {
            // 3. 通过 UDP 无线网卡发往上位机/地面站
            udp_send_packet(tx_buffer, packed_len);
        }

        // 严格维护 20ms (50Hz) 延时
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

/**
 * @brief 100Hz 系统控制与下行指令接收主任务 (10ms 周期)
 */
static void control_loop_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    uint8_t rx_buf[128];
    rc_channels_t rc_raw = {0};
    rc_control_cmd_t robot_cmd = {0};
    downward_frame_t downward_frame;

    float target_joint_angles[3] = {0.0f};

    // 初始化 CPG 步态 (100Hz -> dt=0.01s)
    cpg_gait_init(&s_cpg_handle, 0.01f);
    downward_parser_init(&s_downward_parser);

    ESP_LOGI(TAG, "Control Loop Task Started (100Hz / 10ms)...");

    while (1) {
        // ----------------------------------------------------
        // 1. 检查并接收 UDP 下行指令 (Sim2Real / RL 指令)
        // ----------------------------------------------------
        int rx_len = udp_receive_packet(rx_buf, sizeof(rx_buf));
        bool has_downward_cmd = false;
        if (rx_len > 0) {
            if (downward_parse_packet(rx_buf, rx_len, &downward_frame)) {
                has_downward_cmd = true;
            }
        }

        // ----------------------------------------------------
        // 2. 读取 CRSF 遥控器输入并更新系统状态机
        // ----------------------------------------------------
        crsf_driver_get_channels(&rc_raw);
        rc_controller_process(&rc_raw, &robot_cmd);

        // ----------------------------------------------------
        // 3. 根据状态机执行对应的分支算法逻辑
        // ----------------------------------------------------
        switch (robot_cmd.current_mode) {
            case SYS_MODE_CPG_AUTO: {
                // CPG 游动模式：计算摆尾角度
                float turn_offsets_deg[3] = {
                    robot_cmd.target_yaw_bias_deg, 
                    robot_cmd.target_yaw_bias_deg * 0.5f, 
                    0.0f
                };

                // 更新 CPG 参数
                cpg_gait_set_params_deg(&s_cpg_handle,
                                        robot_cmd.target_amplitude_deg,
                                        robot_cmd.target_frequency_hz,
                                        45.0f,
                                        turn_offsets_deg);

                // 解算 3 关节角度
                cpg_gait_update(&s_cpg_handle, target_joint_angles);
                break;
            }

            case SYS_MODE_RL_SIM2REAL: {
                // RL 上位机透传模式：直接接收下行 Packet 关节角
                if (has_downward_cmd && downward_frame.msg_id == DOWNWARD_MSG_JOINTS_CMD) {
                    for (int i = 0; i < 3; i++) {
                        target_joint_angles[i] = downward_frame.payload.joints_cmd.target_theta[i];
                    }
                }
                break;
            }

            case SYS_MODE_IDLE:
            default: {
                // 归中保护状态
                target_joint_angles[0] = 0.0f;
                target_joint_angles[1] = 0.0f;
                target_joint_angles[2] = 0.0f;
                break;
            }
        }

        // ----------------------------------------------------
        // 4. 将计算得出的 3 关节角度下发给 CAN/TWAI 舵机
        // ----------------------------------------------------
        if (!robot_cmd.emergency_stop) {
            can_servo_set_target_angles_rad(target_joint_angles);
        } else {
            can_servo_disable_all(); // 触发急停断电
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

bool telemetry_task_start(void)
{
    // 1. 创建 50Hz 遥测发送任务 (优先级 4)
    BaseType_t ret1 = xTaskCreate(telemetry_tx_task, "telemetry_tx", 4096, NULL, 4, NULL);

    // 2. 创建 100Hz 闭环控制主任务 (优先级 5, 优先级需高于遥测)
    BaseType_t ret2 = xTaskCreate(control_loop_task, "control_loop", 4096, NULL, 5, NULL);

    return (ret1 == pdPASS && ret2 == pdPASS);
}