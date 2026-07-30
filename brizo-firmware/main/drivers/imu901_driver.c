/**
 * @file imu901_driver.c
 * @brief IMU 驱动实现文件, 适用于 ATK-IMU601,901 (ESP-IDF 5.x 工业级极致优化版)
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-30
 * @version 2.0
 */

#include "imu901_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"
#include <string.h>

static const char *TAG = "IMU_DRIVER";

// ---------- 宏定义与常量规范 ----------
#define UP_BYTE1            0x55
#define UP_BYTE2            0x55
#define UP_BYTE2_ACK        0xAF
#define ATKP_MAX_DATA_SIZE  28
#define UART_BUF_SIZE       (1024)

static const uint16_t kGyroFsrTable[4] = {250, 500, 1000, 2000};
static const uint8_t  kAccFsrTable[4]  = {2, 4, 8, 16};

// ---------- 解包状态机枚举 ----------
typedef enum {
    STATE_WAIT_HEADER1,
    STATE_WAIT_HEADER2,
    STATE_WAIT_MSG_ID,
    STATE_WAIT_LEN,
    STATE_WAIT_DATA,
    STATE_WAIT_CKSUM,
} rx_state_t;

// ---------- 驱动内部全局上下文 ----------
typedef struct {
    portMUX_TYPE     spinlock;
    uart_port_t      uart_num;
    QueueHandle_t    uart_queue;
    TaskHandle_t     task_handle;
    bool             initialized;
    
    // 数据缓存区
    attitude_t       attitude;
    gyroAcc_t        gyro_acc;
    quaternion_t     quat;
    mag_t            mag;
    baro_t           baro;

    // 数据更新 Flag 标志位
    bool             att_updated;
    bool             gyro_acc_updated;
    bool             quat_updated;
    bool             mag_updated;
    bool             baro_updated;

    // 配置参数记录
    regValue_t       param;
} imu_ctx_t;

static imu_ctx_t s_ctx = {
    .spinlock = portMUX_INITIALIZER_UNLOCKED,
    .uart_num = UART_NUM_1,
    .uart_queue = NULL,
    .task_handle = NULL,
    .initialized = false,
    .param = { .gyroFsr = 3, .accFsr = 1, .rate = 4 },
};

// ---------- 私有助手宏：减少 Getter 重复代码 ----------
#define IMU_GET_DATA_ATOMIC(target_ptr, src_field, update_flag) \
    do { \
        if ((target_ptr) == NULL) return false; \
        portENTER_CRITICAL(&s_ctx.spinlock); \
        if (!s_ctx.update_flag) { \
            portEXIT_CRITICAL(&s_ctx.spinlock); \
            return false; \
        } \
        *(target_ptr) = s_ctx.src_field; \
        s_ctx.update_flag = false; \
        portEXIT_CRITICAL(&s_ctx.spinlock); \
        return true; \
    } while(0)

// ---------- 解包状态机 ----------

/**
 * @brief 逐字节状态机解包 (纯净上下文隔离)
 */
bool imu901_unpack_byte(uint8_t ch, atkp_t *out_pkt)
{
    static rx_state_t state = STATE_WAIT_HEADER1;
    static uint8_t cksum = 0;
    static uint8_t data_idx = 0;
    static atkp_t  temp_pkt;

    switch (state) {
        case STATE_WAIT_HEADER1:
            if (ch == UP_BYTE1) {
                state = STATE_WAIT_HEADER2;
                temp_pkt.startByte1 = ch;
                cksum = ch;
            }
            break;

        case STATE_WAIT_HEADER2:
            if (ch == UP_BYTE2 || ch == UP_BYTE2_ACK) {
                state = STATE_WAIT_MSG_ID;
                temp_pkt.startByte2 = ch;
                cksum += ch;
            } else {
                state = STATE_WAIT_HEADER1;
            }
            break;

        case STATE_WAIT_MSG_ID:
            temp_pkt.msgID = ch;
            state = STATE_WAIT_LEN;
            cksum += ch;
            break;

        case STATE_WAIT_LEN:
            if (ch <= ATKP_MAX_DATA_SIZE) {
                temp_pkt.dataLen = ch;
                data_idx = 0;
                state = (ch > 0) ? STATE_WAIT_DATA : STATE_WAIT_CKSUM;
                cksum += ch;
            } else {
                state = STATE_WAIT_HEADER1;
            }
            break;

        case STATE_WAIT_DATA:
            temp_pkt.data[data_idx++] = ch;
            cksum += ch;
            if (data_idx == temp_pkt.dataLen) {
                state = STATE_WAIT_CKSUM;
            }
            break;

        case STATE_WAIT_CKSUM:
            state = STATE_WAIT_HEADER1;
            if (cksum == ch) {
                temp_pkt.checkSum = cksum;
                *out_pkt = temp_pkt; // 校验成功，完整拷贝交付
                return true;
            }
            break;

        default:
            state = STATE_WAIT_HEADER1;
            break;
    }
    return false;
}

/**
 * @brief 数据解析与转换
 */
static void atkp_parse_packet(const atkp_t *packet)
{
    switch (packet->msgID) {
        case UP_ATTITUDE: {
            int16_t roll  = (int16_t)(((uint16_t)packet->data[1] << 8) | packet->data[0]);
            int16_t pitch = (int16_t)(((uint16_t)packet->data[3] << 8) | packet->data[2]);
            int16_t yaw   = (int16_t)(((uint16_t)packet->data[5] << 8) | packet->data[4]);

            attitude_t att = {
                .roll  = (float)roll / 32768.0f * 180.0f,
                .pitch = (float)pitch / 32768.0f * 180.0f,
                .yaw   = (float)yaw / 32768.0f * 180.0f,
            };

            portENTER_CRITICAL(&s_ctx.spinlock);
            s_ctx.attitude = att;
            s_ctx.att_updated = true;
            portEXIT_CRITICAL(&s_ctx.spinlock);
            break;
        }
        case UP_QUAT: {
            int16_t q0 = (int16_t)(((uint16_t)packet->data[1] << 8) | packet->data[0]);
            int16_t q1 = (int16_t)(((uint16_t)packet->data[3] << 8) | packet->data[2]);
            int16_t q2 = (int16_t)(((uint16_t)packet->data[5] << 8) | packet->data[4]);
            int16_t q3 = (int16_t)(((uint16_t)packet->data[7] << 8) | packet->data[6]);

            quaternion_t quat = {
                .q0 = (float)q0 / 32768.0f,
                .q1 = (float)q1 / 32768.0f,
                .q2 = (float)q2 / 32768.0f,
                .q3 = (float)q3 / 32768.0f,
            };

            portENTER_CRITICAL(&s_ctx.spinlock);
            s_ctx.quat = quat;
            s_ctx.quat_updated = true;
            portEXIT_CRITICAL(&s_ctx.spinlock);
            break;
        }
        case UP_GYROACCDATA: {
            gyroAcc_t ga;
            ga.acc[0]  = (int16_t)(((uint16_t)packet->data[1] << 8) | packet->data[0]);
            ga.acc[1]  = (int16_t)(((uint16_t)packet->data[3] << 8) | packet->data[2]);
            ga.acc[2]  = (int16_t)(((uint16_t)packet->data[5] << 8) | packet->data[4]);
            ga.gyro[0] = (int16_t)(((uint16_t)packet->data[7] << 8) | packet->data[6]);
            ga.gyro[1] = (int16_t)(((uint16_t)packet->data[9] << 8) | packet->data[8]);
            ga.gyro[2] = (int16_t)(((uint16_t)packet->data[11] << 8) | packet->data[10]);

            float acc_scale  = (float)kAccFsrTable[s_ctx.param.accFsr & 0x03] / 32768.0f;
            float gyro_scale = (float)kGyroFsrTable[s_ctx.param.gyroFsr & 0x03] / 32768.0f;

            ga.faccG[0]  = ga.acc[0] * acc_scale;
            ga.faccG[1]  = ga.acc[1] * acc_scale;
            ga.faccG[2]  = ga.acc[2] * acc_scale;
            ga.fgyroD[0] = ga.gyro[0] * gyro_scale;
            ga.fgyroD[1] = ga.gyro[1] * gyro_scale;
            ga.fgyroD[2] = ga.gyro[2] * gyro_scale;

            portENTER_CRITICAL(&s_ctx.spinlock);
            s_ctx.gyro_acc = ga;
            s_ctx.gyro_acc_updated = true;
            portEXIT_CRITICAL(&s_ctx.spinlock);
            break;
        }
        case UP_MAGDATA: {
            mag_t mag;
            mag.mag[0] = (int16_t)(((uint16_t)packet->data[1] << 8) | packet->data[0]);
            mag.mag[1] = (int16_t)(((uint16_t)packet->data[3] << 8) | packet->data[2]);
            mag.mag[2] = (int16_t)(((uint16_t)packet->data[5] << 8) | packet->data[4]);
            int16_t temp = (int16_t)(((uint16_t)packet->data[7] << 8) | packet->data[6]);
            mag.temp = (float)temp / 100.0f;

            portENTER_CRITICAL(&s_ctx.spinlock);
            s_ctx.mag = mag;
            s_ctx.mag_updated = true;
            portEXIT_CRITICAL(&s_ctx.spinlock);
            break;
        }
        case UP_BARODATA: {
            uint32_t press_u = ((uint32_t)packet->data[3] << 24) | ((uint32_t)packet->data[2] << 16) |
                               ((uint32_t)packet->data[1] << 8)  | packet->data[0];
            uint32_t alt_u   = ((uint32_t)packet->data[7] << 24) | ((uint32_t)packet->data[6] << 16) |
                               ((uint32_t)packet->data[5] << 8)  | packet->data[4];
            int16_t temp     = (int16_t)(((uint16_t)packet->data[9] << 8) | packet->data[8]);

            baro_t baro = {
                .pressure = (int32_t)press_u,
                .altitude = (int32_t)alt_u,
                .temp     = (float)temp / 100.0f,
            };

            portENTER_CRITICAL(&s_ctx.spinlock);
            s_ctx.baro = baro;
            s_ctx.baro_updated = true;
            portEXIT_CRITICAL(&s_ctx.spinlock);
            break;
        }
        default:
            break;
    }
}

// ---------- 寄存器写入接口 ----------

static inline void imu_uart_send(const uint8_t *data, size_t len)
{
    uart_write_bytes(s_ctx.uart_num, (const char*)data, len);
}

static void atkp_write_reg(uint8_t reg, uint16_t data, uint8_t datalen)
{
    uint8_t buf[7] = {0x55, 0xAF, reg, datalen, data & 0xFF};

    if (datalen == 2) {
        buf[5] = (data >> 8) & 0xFF;
        buf[6] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5];
        imu_uart_send(buf, 7);
    } else {
        buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
        imu_uart_send(buf, 6);
    }
}

static void imu901_configure_default(void)
{
    atkp_write_reg(REG_GYROFSR, 0x03, 1); // 2000 deg/s
    atkp_write_reg(REG_ACCFSR,  0x01, 1); // 4G
    atkp_write_reg(REG_UPSET,   0x00, 1); // 主动上报
    atkp_write_reg(REG_UPRATE,  0x04, 1); // 50Hz 上报率
    atkp_write_reg(REG_SAVE,    0x00, 1); // 保存至 Flash

    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "IMU901 配置指令已下发完毕");
}

// ---------- 后台事件响应 Task (零 CPU 轮询占用) ----------

static void imu_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t rx_buf[128];
    atkp_t parsed_packet;

    while (1) {
        // 阻塞等待 UART 硬件事件队列，产生数据中断才被唤醒，极度省电高效
        if (xQueueReceive(s_ctx.uart_queue, (void *)&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    while (event.size > 0) {
                        int chunk = (event.size > sizeof(rx_buf)) ? sizeof(rx_buf) : event.size;
                        int len = uart_read_bytes(s_ctx.uart_num, rx_buf, chunk, portMAX_DELAY);
                        
                        for (int i = 0; i < len; i++) {
                            if (imu901_unpack_byte(rx_buf[i], &parsed_packet)) {
                                if (parsed_packet.startByte2 == UP_BYTE2) {
                                    atkp_parse_packet(&parsed_packet);
                                }
                            }
                        }
                        event.size -= len;
                    }
                    break;

                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART 缓冲区溢出，正在刷新...");
                    uart_flush_input(s_ctx.uart_num);
                    xQueueReset(s_ctx.uart_queue);
                    break;

                default:
                    break;
            }
        }
    }
}

// ---------- 公共 API 接口实现 ----------

void imu_init(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, int baud_rate)
{
    if (s_ctx.initialized) {
        ESP_LOGW(TAG, "IMU 驱动已初始化，跳过重构");
        return;
    }

    s_ctx.uart_num = uart_num;

    const uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // 安装驱动并开启中断事件队列 (20 队列容量)
    ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_BUF_SIZE, UART_BUF_SIZE, 20, &s_ctx.uart_queue, 0));

    // 给模块物理稳定上电留出缓冲时间
    vTaskDelay(pdMS_TO_TICKS(200));

    // 下发配置参数
    imu901_configure_default();

    // 绑定建立基于 Event 的解包任务
    xTaskCreate(imu_event_task, "imu_event_task", 3072, NULL, 12, &s_ctx.task_handle);

    s_ctx.initialized = true;
    ESP_LOGI(TAG, "IMU901 驱动在 UART%d 上初始化成功 (TX:%d, RX:%d)", uart_num, tx_pin, rx_pin);
}

bool imu_get_attitude(attitude_t *att)
{
    IMU_GET_DATA_ATOMIC(att, attitude, att_updated);
}

bool imu_get_gyro_acc(gyroAcc_t *gyro_acc)
{
    IMU_GET_DATA_ATOMIC(gyro_acc, gyro_acc, gyro_acc_updated);
}

bool imu_get_quaternion(quaternion_t *quat)
{
    IMU_GET_DATA_ATOMIC(quat, quat, quat_updated);
}

bool imu_get_mag(mag_t *mag)
{
    IMU_GET_DATA_ATOMIC(mag, mag, mag_updated);
}

bool imu_get_baro(baro_t *baro)
{
    IMU_GET_DATA_ATOMIC(baro, baro, baro_updated);
}