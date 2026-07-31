/**
 * @file main.c
 * @brief 机器鱼主控制程序 (ESP-IDF FreeRTOS 多任务架构)
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-31
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"

/* ---------------- 驱动层头文件 ---------------- */
#include "drivers/can_servo_driver.h"
#include "drivers/imu901_driver.h"
#include "drivers/crsf_driver.h"
#include "drivers/udp_driver.h"
#include "drivers/adc_driver.h"
#include "drivers/led_driver.h"

/* ---------------- 应用层头文件 ---------------- */
#include "app/cpg_gait.h"
#include "app/rc_controller.h"

static const char *TAG = "MAIN_APP";

/* ---------------- 引脚与硬件配置 ---------------- */
#define PIN_CAN_TX      GPIO_NUM_4
#define PIN_CAN_RX      GPIO_NUM_5

#define PIN_IMU_TX      GPIO_NUM_6
#define PIN_IMU_RX      GPIO_NUM_7
#define IMU_UART_PORT   UART_NUM_1

#define PIN_CRSF_RX     GPIO_NUM_37
#define CRSF_UART_PORT  UART_NUM_2

#define PIN_ADC_TEMP    GPIO_NUM_16
#define PIN_LED_STATUS  GPIO_NUM_17

/* ---------------- 网络配置 ---------------- */
#define UDP_LOCAL_PORT  8080
#define UDP_REMOTE_PORT 8080

/* ---------------- UDP 数据协议结构体定义 ---------------- */
typedef struct {
    float theta[CPG_JOINTS_NUM];    /*!< 3关节期望角度 (rad) */
} __attribute__((packed)) downward_info_t;

typedef struct {
    uint8_t id[CPG_JOINTS_NUM];     /*!< 舵机 ID */
    float theta[CPG_JOINTS_NUM];    /*!< 舵机当前角度 (deg) */
    float vin[CPG_JOINTS_NUM];      /*!< 舵机当前电流 (A) */
    int16_t temp[CPG_JOINTS_NUM];   /*!< 温度 */
    float rpy[3];                   /*!< Roll, Pitch, Yaw (deg) */
    float acc[3];                   /*!< 加速度 (g) */
    float gyro[3];                  /*!< 角速度 (deg/s) */
} __attribute__((packed)) upward_info_t;

/* ---------------- 全局变量与线程同步 ---------------- */
static cpg_handle_t g_cpg_handle;
static SemaphoreHandle_t g_data_mutex = NULL; // 数据互斥锁

typedef struct {
    rc_input_data_t rc_input;
    downward_info_t downward_cmd;
    upward_info_t upward_telemetry;
    control_source_e control_src;
} robot_shared_context_t;

static robot_shared_context_t g_robot_ctx;

/* ========================================================================= */
/* 任务 1：CPG 解算与舵机控制主任务 (50Hz 强实时，运行在 Core 1)            */
/* ========================================================================= */
static void cpg_control_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms (50Hz) 严格控制周期

    float joint_angles_rad[CPG_JOINTS_NUM] = {0};

    ESP_LOGI(TAG, "CPG Control Task initialized on Core %d", xPortGetCoreID());

    while (1) {
        // 1. 读取遥控数据副本
        rc_input_data_t rc_in;
        xSemaphoreTake(g_data_mutex, portMAX_DELAY);
        rc_in = g_robot_ctx.rc_input;
        xSemaphoreGive(g_data_mutex);

        // 2. 更新遥控映射逻辑及 CPG 参数
        control_source_e current_src = rc_controller_update(&rc_in, &g_cpg_handle);

        // 3. CPG 积分求解或重放轨迹更新
        cpg_gait_update(&g_cpg_handle, joint_angles_rad);

        // 4. 更新全局控制模式状态
        xSemaphoreTake(g_data_mutex, portMAX_DELAY);
        g_robot_ctx.control_src = current_src;
        xSemaphoreGive(g_data_mutex);

        // 5. 下发舵机目标控制角度
        for (int i = 0; i < CPG_JOINTS_NUM; i++) {
            uint8_t node_id = 0x25 + i; // 0x25, 0x26, 0x27
            float target_deg = 0.0f;

            if (current_src == CONTROL_SRC_DOWNWARD_UDP) {
                // 上位机 UDP 命令控制模式
                xSemaphoreTake(g_data_mutex, portMAX_DELAY);
                float rad_val = g_robot_ctx.downward_cmd.theta[i];
                xSemaphoreGive(g_data_mutex);

                // 限幅 [-1.047, 1.047] rad (即 ±60°)
                if (rad_val < -1.047f) rad_val = -1.047f;
                if (rad_val > 1.047f) rad_val = 1.047f;
                target_deg = RAD2DEG(rad_val);
            } else {
                // CPG / AHC / 重放模式
                target_deg = RAD2DEG(joint_angles_rad[i]);
            }

            // 非阻塞/快速 CAN 发送
            can_servo_set_position(node_id, target_deg);
        }

        // 翻转运行指示灯
        led_toggle();

        // 绝对延时以维持精确 50Hz 周期
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/* ========================================================================= */
/* 任务 2：IMU901 姿态与 CRSF 遥控数据刷新任务 (Core 1)                      */
/* ========================================================================= */
static void imu_crsf_task(void *pvParameters)
{
    crsf_channels_t crsf_ch_data;
    attitude_t att;
    gyroAcc_t gyro_acc;

    ESP_LOGI(TAG, "IMU & CRSF Task running on Core %d", xPortGetCoreID());

    while (1) {
        // 1. 读取 CRSF 摇杆通道数据
        if (crsf_get_channels(&crsf_ch_data)) {
            xSemaphoreTake(g_data_mutex, portMAX_DELAY);
            for (int i = 0; i < CRSF_CHANNEL_COUNT; i++) {
                uint16_t val = crsf_ch_data.ch[i];
                if (val < 1000) val = 1000;
                if (val > 2000) val = 2000;
                // 反转并限幅映射
                g_robot_ctx.rc_input.crsf_ch[i] = 3000 - val;
            }
            xSemaphoreGive(g_data_mutex);
        }

        // 2. 读取 IMU901 姿态角与传感器原始值
        if (imu_get_attitude(&att)) {
            xSemaphoreTake(g_data_mutex, portMAX_DELAY);
            g_robot_ctx.rc_input.current_yaw_deg = att.yaw;
            
            g_robot_ctx.upward_telemetry.rpy[0] = att.roll;
            g_robot_ctx.upward_telemetry.rpy[1] = att.pitch;
            g_robot_ctx.upward_telemetry.rpy[2] = att.yaw;
            xSemaphoreGive(g_data_mutex);
        }

        if (imu_get_gyro_acc(&gyro_acc)) {
            xSemaphoreTake(g_data_mutex, portMAX_DELAY);
            for (int i = 0; i < 3; i++) {
                g_robot_ctx.upward_telemetry.acc[i] = gyro_acc.faccG[i];
                g_robot_ctx.upward_telemetry.gyro[i] = gyro_acc.fgyroD[i];
            }
            xSemaphoreGive(g_data_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz 采样频率
    }
}

/* ========================================================================= */
/* 任务 3：UDP 遥测数据打包上报与 CAN 反馈查询 (Core 0)                      */
/* ========================================================================= */
static void telemetry_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms 上报频率

    while (1) {
        upward_info_t send_pkt;
        memset(&send_pkt, 0, sizeof(upward_info_t));

        // 1. 查询舵机反馈 (角度、电流、内部温度)
        for (int i = 0; i < CPG_JOINTS_NUM; i++) {
            uint8_t node_id = 0x25 + i;
            float pos = 0.0f;
            float curr = 0.0f;
            int16_t servo_temp = 0;

            can_servo_get_position(node_id, &pos);
            can_servo_get_current(node_id, &curr, &servo_temp);

            send_pkt.id[i] = node_id;
            send_pkt.theta[i] = pos;
            send_pkt.vin[i] = curr;
            send_pkt.temp[i] = servo_temp;
        }

        // 2. 组装传感器数据
        int adc_mv = adc_read_millvoltage();
        xSemaphoreTake(g_data_mutex, portMAX_DELAY);
        memcpy(send_pkt.rpy, g_robot_ctx.upward_telemetry.rpy, sizeof(send_pkt.rpy));
        memcpy(send_pkt.acc, g_robot_ctx.upward_telemetry.acc, sizeof(send_pkt.acc));
        memcpy(send_pkt.gyro, g_robot_ctx.upward_telemetry.gyro, sizeof(send_pkt.gyro));
        xSemaphoreGive(g_data_mutex);

        // 3. 网络回传
        if (udp_is_connected()) {
            udp_send_packet((const uint8_t *)&send_pkt, sizeof(upward_info_t));
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/* ========================================================================= */
/* 任务 4：UDP 下行控制命令接收任务 (Core 0)                                 */
/* ========================================================================= */
static void udp_rx_task(void *pvParameters)
{
    downward_info_t recv_cmd;

    while (1) {
        int rx_bytes = udp_receive_packet((uint8_t *)&recv_cmd, sizeof(downward_info_t));
        if (rx_bytes == sizeof(downward_info_t)) {
            xSemaphoreTake(g_data_mutex, portMAX_DELAY);
            g_robot_ctx.downward_cmd = recv_cmd;
            xSemaphoreGive(g_data_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* ========================================================================= */
/* 主函数入口 app_main                                                      */
/* ========================================================================= */
void app_main(void)
{
    ESP_LOGI(TAG, "Initializing BRIZO Micro-Robotic Firmware...");

    // 1. 初始化互斥锁
    g_data_mutex = xSemaphoreCreateMutex();
    configASSERT(g_data_mutex != NULL);

    // 2. 底层外设驱动初始化
    led_init(PIN_LED_STATUS);
    adc_init(PIN_ADC_TEMP);
    
    if (!can_servo_init(PIN_CAN_TX, PIN_CAN_RX)) {
        ESP_LOGE(TAG, "TWAI CAN Servo Init Failed!");
    }

    crsf_init(CRSF_UART_PORT, PIN_CRSF_RX, GPIO_NUM_NC);
    imu_init(IMU_UART_PORT, PIN_IMU_TX, PIN_IMU_RX, 115200);

    // WiFi & UDP 配置
    wifi_udp_info_t udp_cfg[] = {
        {
            .ssid = "BRIZO_AP",
            .password = "12345678",
            .local_port = UDP_LOCAL_PORT,
            .remote_port = UDP_REMOTE_PORT,
            .remote_ip = 0 // 自动取 Gateway IP
        }
    };
    udp_driver_init(udp_cfg, 1);

    // 3. 控制算法与手柄映射初始化
    cpg_gait_init(&g_cpg_handle, 0.02f); // 20ms dt
    rc_controller_init();

    // 4. 创建 FreeRTOS 任务并绑定核心
    // 控制与传感器采集绑定至 Core 1 (避开 Wi-Fi 协议栈中断)
    xTaskCreatePinnedToCore(cpg_control_task, "cpg_ctrl", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(imu_crsf_task,   "imu_crsf", 3072, NULL, 4, NULL, 1);

    // 网络与遥测通信绑定至 Core 0
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(udp_rx_task,    "udp_rx",    3072, NULL, 2, NULL, 0);

    ESP_LOGI(TAG, "FreeRTOS Scheduler initialised successfully.");
}