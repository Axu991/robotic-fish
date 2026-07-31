/**
 * @file robot_app.c
 * @brief 机器鱼应用任务实现
 */

#include "robot_app.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "adc_driver.h"
#include "can_servo_driver.h"
#include "crsf_driver.h"
#include "imu901_driver.h"
#include "led_driver.h"
#include "udp_driver.h"

#include "downward_parser.h"
#include "upward_packer.h"

#include "cpg_gait.h"
#include "rc_controller.h"

static const char *TAG = "ROBOT_APP";

#define CONTROL_PERIOD_MS       20
#define SENSOR_PERIOD_MS        10
#define TELEMETRY_PERIOD_MS     20
#define UDP_COMMAND_TIMEOUT_US  500000LL
#define RC_INPUT_TIMEOUT_US     500000LL
#define JOINT_LIMIT_RAD         1.047f
#define SERVO_NODE_ID_BASE      0x25
#define APP_TASK_COUNT          4

_Static_assert(CPG_JOINTS_NUM == DOWNWARD_SERVO_NUMS,
               "CPG and downward protocol joint counts must match");
_Static_assert(CPG_JOINTS_NUM == SERVO_NUMS,
               "CPG and upward protocol joint counts must match");

typedef struct {
    rc_input_data_t rc_input;
    downward_payload_joints_cmd_t downward_cmd;
    upward_payload_telemetry_t upward_telemetry;
    control_source_e control_src;
    int64_t last_rc_update_us;
    int64_t last_udp_update_us;
    bool rc_valid;
    bool udp_cmd_valid;
} robot_shared_context_t;

static cpg_handle_t s_cpg_handle;
static robot_shared_context_t s_robot_ctx;
static SemaphoreHandle_t s_data_mutex;
static TaskHandle_t s_task_handles[APP_TASK_COUNT];
static bool s_started;

static float clamp_joint_rad(float angle_rad)
{
    if (!isfinite(angle_rad)) return 0.0f;
    if (angle_rad < -JOINT_LIMIT_RAD) return -JOINT_LIMIT_RAD;
    if (angle_rad > JOINT_LIMIT_RAD) return JOINT_LIMIT_RAD;
    return angle_rad;
}

static bool create_task_checked(TaskFunction_t task, const char *name,
                                uint32_t stack_size, UBaseType_t priority,
                                BaseType_t core_id, TaskHandle_t *handle)
{
    BaseType_t result = xTaskCreatePinnedToCore(
        task, name, stack_size, NULL, priority, handle, core_id);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task %s", name);
        return false;
    }
    return true;
}

static void delete_created_tasks(void)
{
    for (int i = 0; i < APP_TASK_COUNT; i++) {
        if (s_task_handles[i] != NULL) {
            vTaskDelete(s_task_handles[i]);
            s_task_handles[i] = NULL;
        }
    }
}

static void cpg_control_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
    float joint_angles_rad[CPG_JOINTS_NUM] = {0};

    ESP_LOGI(TAG, "CPG control task running on Core %d", xPortGetCoreID());

    while (1) {
        robot_shared_context_t snapshot;
        xSemaphoreTake(s_data_mutex, portMAX_DELAY);
        snapshot = s_robot_ctx;
        xSemaphoreGive(s_data_mutex);

        int64_t now_us = esp_timer_get_time();
        bool rc_fresh = snapshot.rc_valid
                     && (now_us - snapshot.last_rc_update_us <= RC_INPUT_TIMEOUT_US);
        bool udp_fresh = snapshot.udp_cmd_valid
                      && (now_us - snapshot.last_udp_update_us <= UDP_COMMAND_TIMEOUT_US);

        control_source_e current_src = snapshot.control_src;
        if (rc_fresh) {
            current_src = rc_controller_update(&snapshot.rc_input, &s_cpg_handle);
        }

        cpg_gait_update(&s_cpg_handle, joint_angles_rad);

        xSemaphoreTake(s_data_mutex, portMAX_DELAY);
        s_robot_ctx.control_src = current_src;
        xSemaphoreGive(s_data_mutex);

        for (int i = 0; i < CPG_JOINTS_NUM; i++) {
            float target_rad = 0.0f;
            if (current_src == CONTROL_SRC_DOWNWARD_UDP) {
                target_rad = udp_fresh ? snapshot.downward_cmd.target_theta[i] : 0.0f;
            } else if (rc_fresh) {
                target_rad = joint_angles_rad[i];
            }

            can_servo_set_position(SERVO_NODE_ID_BASE + i,
                                   RAD2DEG(clamp_joint_rad(target_rad)));
        }

        led_toggle();
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

static void imu_crsf_task(void *pvParameters)
{
    crsf_channels_t crsf_channels;
    attitude_t attitude;
    gyroAcc_t gyro_acc;

    ESP_LOGI(TAG, "IMU/CRSF task running on Core %d", xPortGetCoreID());

    while (1) {
        if (crsf_get_channels(&crsf_channels)) {
            xSemaphoreTake(s_data_mutex, portMAX_DELAY);
            for (int i = 0; i < CRSF_CHANNEL_COUNT; i++) {
                uint16_t value = crsf_channels.ch[i];
                if (value < 1000) value = 1000;
                if (value > 2000) value = 2000;
                s_robot_ctx.rc_input.crsf_ch[i] = 3000 - value;
            }
            s_robot_ctx.rc_valid = true;
            s_robot_ctx.last_rc_update_us = esp_timer_get_time();
            xSemaphoreGive(s_data_mutex);
        }

        if (imu_get_attitude(&attitude)) {
            xSemaphoreTake(s_data_mutex, portMAX_DELAY);
            s_robot_ctx.rc_input.current_yaw_deg = attitude.yaw;
            s_robot_ctx.upward_telemetry.rpy[0] = attitude.roll;
            s_robot_ctx.upward_telemetry.rpy[1] = attitude.pitch;
            s_robot_ctx.upward_telemetry.rpy[2] = attitude.yaw;
            xSemaphoreGive(s_data_mutex);
        }

        if (imu_get_gyro_acc(&gyro_acc)) {
            xSemaphoreTake(s_data_mutex, portMAX_DELAY);
            for (int i = 0; i < 3; i++) {
                s_robot_ctx.upward_telemetry.acc[i] = gyro_acc.faccG[i];
                s_robot_ctx.upward_telemetry.gyro[i] = gyro_acc.fgyroD[i];
            }
            xSemaphoreGive(s_data_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

static void telemetry_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(TELEMETRY_PERIOD_MS);
    upward_payload_telemetry_t telemetry = {0};
    uint8_t tx_buffer[UPWARD_MAX_PACKET_LEN];
    int next_servo = 0;
    bool read_position = true;

    while (1) {
        uint8_t node_id = SERVO_NODE_ID_BASE + next_servo;
        if (read_position) {
            float position_deg = 0.0f;
            if (can_servo_get_position(node_id, &position_deg)) {
                telemetry.theta[next_servo] = DEG2RAD(position_deg);
            }
        } else {
            float current_a = 0.0f;
            if (can_servo_get_current(node_id, &current_a, NULL)) {
                telemetry.vin[next_servo] = current_a;
            }
        }
        telemetry.id[next_servo] = node_id;
        read_position = !read_position;
        if (read_position) {
            next_servo = (next_servo + 1) % CPG_JOINTS_NUM;
        }

        int adc_mv = adc_read_millvoltage();
        int32_t temperature = (adc_mv >= 0)
                            ? (int32_t)((float)adc_mv / 5.6f * (5.6f + 10.0f))
                            : -1;

        xSemaphoreTake(s_data_mutex, portMAX_DELAY);
        memcpy(telemetry.rpy, s_robot_ctx.upward_telemetry.rpy, sizeof(telemetry.rpy));
        memcpy(telemetry.acc, s_robot_ctx.upward_telemetry.acc, sizeof(telemetry.acc));
        memcpy(telemetry.gyro, s_robot_ctx.upward_telemetry.gyro, sizeof(telemetry.gyro));
        xSemaphoreGive(s_data_mutex);

        for (int i = 0; i < CPG_JOINTS_NUM; i++) {
            telemetry.temp[i] = temperature;
        }

        if (udp_is_connected()) {
            size_t tx_len = 0;
            if (upward_pack_telemetry(&telemetry, tx_buffer, &tx_len)) {
                udp_send_packet(tx_buffer, tx_len);
            }
        }

        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

static void udp_rx_task(void *pvParameters)
{
    uint8_t rx_buffer[DOWNWARD_MAX_PACKET_LEN];
    downward_frame_t frame;

    while (1) {
        int rx_bytes = udp_receive_packet(rx_buffer, sizeof(rx_buffer));
        if (rx_bytes < 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (rx_bytes == 0) continue;

        if (!downward_parse_packet(rx_buffer, rx_bytes, &frame)) {
            ESP_LOGW(TAG, "Discarded invalid downward packet (%d bytes)", rx_bytes);
            continue;
        }
        if (frame.msg_id != DOWNWARD_MSG_JOINTS_CMD
                || frame.payload_len != sizeof(downward_payload_joints_cmd_t)) {
            ESP_LOGW(TAG, "Ignored downward frame: id=0x%02X len=%u",
                     frame.msg_id, frame.payload_len);
            continue;
        }

        downward_payload_joints_cmd_t command = frame.payload.joints_cmd;
        bool valid = true;
        for (int i = 0; i < CPG_JOINTS_NUM; i++) {
            if (!isfinite(command.target_theta[i])
                    || !isfinite(command.target_speed[i])) {
                valid = false;
                break;
            }
            command.target_theta[i] = clamp_joint_rad(command.target_theta[i]);
        }
        if (!valid) {
            ESP_LOGW(TAG, "Discarded non-finite downward command");
            continue;
        }

        xSemaphoreTake(s_data_mutex, portMAX_DELAY);
        s_robot_ctx.downward_cmd = command;
        s_robot_ctx.udp_cmd_valid = true;
        s_robot_ctx.last_udp_update_us = esp_timer_get_time();
        xSemaphoreGive(s_data_mutex);
    }
}

bool robot_app_start(void)
{
    if (s_started) return true;

    memset(&s_robot_ctx, 0, sizeof(s_robot_ctx));
    memset(s_task_handles, 0, sizeof(s_task_handles));

    s_data_mutex = xSemaphoreCreateMutex();
    if (s_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create application data mutex");
        return false;
    }

    const uint16_t default_rc[CRSF_CHANNEL_COUNT] = {
        1500, 1500, 2000, 1500, 2000, 2000, 2000, 2000, 2000, 2000
    };
    memcpy(s_robot_ctx.rc_input.crsf_ch, default_rc, sizeof(default_rc));
    s_robot_ctx.control_src = CONTROL_SRC_DOWNWARD_UDP;

    cpg_gait_init(&s_cpg_handle, CONTROL_PERIOD_MS / 1000.0f);
    rc_controller_init();

    bool ok = true;
    ok &= create_task_checked(cpg_control_task, "cpg_ctrl", 4096, 5, 1,
                              &s_task_handles[0]);
    ok &= create_task_checked(imu_crsf_task, "imu_crsf", 3072, 4, 1,
                              &s_task_handles[1]);
    ok &= create_task_checked(telemetry_task, "telemetry", 4096, 3, 0,
                              &s_task_handles[2]);
    ok &= create_task_checked(udp_rx_task, "udp_rx", 3072, 2, 0,
                              &s_task_handles[3]);

    if (!ok) {
        delete_created_tasks();
        vSemaphoreDelete(s_data_mutex);
        s_data_mutex = NULL;
        return false;
    }

    s_started = true;
    ESP_LOGI(TAG, "Robot application tasks started");
    return true;
}
