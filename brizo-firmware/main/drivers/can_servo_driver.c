/** 
 * @file can_servo_driver.c
 * @brief CAN 舵机驱动实现，舵机型号为 KingmaxBLS4510S
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-30
 * @version 1.0
*/

#include "can_servo_driver.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "CAN_SERVO";

static bool twai_initialized = false;
static SemaphoreHandle_t s_twai_mutex = NULL;

#define TWAI_TX_TIMEOUT_TICKS       1
#define TWAI_RESPONSE_TIMEOUT_TICKS 1

// 内部辅助函数：发送 CAN 消息（带重试）
static bool twai_transmit_with_retry(const twai_message_t *msg, TickType_t timeout_ticks)
{
    if (!twai_initialized) 
    {
        ESP_LOGE(TAG, "TWAI driver not initialized");
        return false;
    }
    esp_err_t ret = twai_transmit(msg, timeout_ticks);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to transmit TWAI message: %s", esp_err_to_name(ret));
        return false;
    }
    return true;
}

// 内部辅助函数：接收 CAN 消息（带超时）
static bool twai_receive_with_timeout(twai_message_t *msg, TickType_t timeout_ticks)
{
    if (!twai_initialized)
    {
        ESP_LOGE(TAG, "TWAI driver not initialized");
        return false;
    }
    esp_err_t ret = twai_receive(msg, timeout_ticks);
    if (ret != ESP_OK)
    {
        return false;
    }
    return true;
}

// ======================公开接口======================

bool can_servo_init(gpio_num_t tx_pin, gpio_num_t rx_pin)
{   
    if (s_twai_mutex == NULL) {
        s_twai_mutex = xSemaphoreCreateMutex();
        if (s_twai_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create TWAI mutex");
            return false;
        }
    }
    if (twai_initialized)
    {   
        ESP_LOGW(TAG, "TWAI driver already initialized");
        twai_stop();
        twai_driver_uninstall();
        twai_initialized = false;
        vTaskDelay(pdMS_TO_TICKS(100)); // 等待一段时间确保驱动卸载完成
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(ret));
        return false;
    }

    ret = twai_start();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(ret));
        return false;
    }

    twai_initialized = true;
    ESP_LOGI(TAG, "TWAI driver initialized on TX pin %d, RX pin %d", tx_pin, rx_pin);
    return true;
}

bool can_servo_set_position(uint8_t node_id, float position_deg)
{
    // 将角度转换为0.1度单位（int16)
    int16_t pos = (int16_t)(position_deg * 10.0f);

    twai_message_t tx_msg = 
    {
        .identifier = 0x0600 + node_id,
        .data_length_code = 8,
        .data = 
        {
            0x22, 0x03, 0x60, 0x00,
            (uint8_t)(pos & 0xFF), 
            (uint8_t)((pos >> 8) & 0xFF),
            0x00, 0x00
        }
    };
    xSemaphoreTake(s_twai_mutex, portMAX_DELAY);
    bool ok = twai_transmit_with_retry(&tx_msg, TWAI_TX_TIMEOUT_TICKS);
    xSemaphoreGive(s_twai_mutex);
    return ok;
}

bool can_servo_get_position(uint8_t node_id, float *position)
{   
    if (position == NULL)
    {
        ESP_LOGE(TAG, "Position pointer is NULL");
        return false;
    }

    xSemaphoreTake(s_twai_mutex, portMAX_DELAY);

    // 发送读取位置命令
    twai_message_t tx_msg = 
    {
        .identifier = 0x0600 + node_id,
        .data_length_code = 8,
        .data = {0x40, 0x02, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}
    };

    if (!twai_transmit_with_retry(&tx_msg, TWAI_TX_TIMEOUT_TICKS))
    {
        xSemaphoreGive(s_twai_mutex);
        return false;
    }

    // 接收响应(预期 ID: 0x0580 + node_id)
    twai_message_t rx_msg;
    TickType_t start_tick = xTaskGetTickCount();
    const TickType_t timeout = TWAI_RESPONSE_TIMEOUT_TICKS;
    while (xTaskGetTickCount() - start_tick < timeout)
    {
        if (twai_receive_with_timeout(&rx_msg, TWAI_RESPONSE_TIMEOUT_TICKS))
        {
            if (rx_msg.identifier == (0x0580 + node_id) && 
                rx_msg.data_length_code == 8 &&
                rx_msg.data[0] == 0x4B &&
                rx_msg.data[1] == 0x02 &&
                rx_msg.data[2] == 0x60)
                {       
                    //解析位置
                    int16_t raw = (int16_t)(rx_msg.data[5] << 8 | rx_msg.data[4]);
                    *position = raw / 10.0f; // 转换回度
                    xSemaphoreGive(s_twai_mutex);
                    return true;
                }   
        }
    }
    ESP_LOGE(TAG, "Timeout waiting for position response from node 0x%02X", node_id);
    xSemaphoreGive(s_twai_mutex);
    return false;
}

bool can_servo_get_current(uint8_t node_id, float *current, int16_t *temperature)
{
    if (current == NULL) return false;
    xSemaphoreTake(s_twai_mutex, portMAX_DELAY);
    
    // 发送读取电流命令
    twai_message_t tx_msg = 
    {
        .identifier = 0x0600 + node_id,
        .data_length_code = 8,
        .data = {0x40, 0x05, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}
    };

    if (!twai_transmit_with_retry(&tx_msg, TWAI_TX_TIMEOUT_TICKS))
    {
        xSemaphoreGive(s_twai_mutex);
        return false;
    }

    // 接收响应(预期 ID: 0x0580 + node_id)
    twai_message_t rx_msg;
    TickType_t start_tick = xTaskGetTickCount();
    const TickType_t timeout = TWAI_RESPONSE_TIMEOUT_TICKS;
    while (xTaskGetTickCount() - start_tick < timeout)
    {
        if (twai_receive_with_timeout(&rx_msg, TWAI_RESPONSE_TIMEOUT_TICKS))
        {
            if (rx_msg.identifier == (0x0580 + node_id) && 
                rx_msg.data_length_code == 8 &&
                rx_msg.data[0] == 0x43 &&
                rx_msg.data[1] == 0x05 &&
                rx_msg.data[2] == 0x60)
                {       
                    //解析电流和温度
                    int16_t raw = (int16_t)(rx_msg.data[5] << 8 | rx_msg.data[4]);
                    *current = raw / 100.0f; // 转换为安培
                    if (temperature != NULL)
                    {
                        *temperature = (int16_t)rx_msg.data[6]; // 温度直接使用原始值
                    }
                    xSemaphoreGive(s_twai_mutex);
                    return true;
                }   
        }
    }
    ESP_LOGE(TAG, "Timeout waiting for current response from node 0x%02X", node_id);
    xSemaphoreGive(s_twai_mutex);
    return false;
}

int can_servo_read_all_feedback(servo_feedback_t *fb, int count)
{
    int success = 0;
    for (int i = 0; i < count; ++i)
    {
        // 读取位置
        float pos;
        if (can_servo_get_position(fb[i].id, &pos))
        {
            fb[i].position = pos;

            // 读取电流和温度
            float cur;
            int16_t temp;
            if (can_servo_get_current(fb[i].id, &cur, &temp)){
                fb[i].current = cur;
                fb[i].temperature = temp;
                success++;
            }
            else{
                fb[i].current = 0.0f; // 如果读取电流失败，设置为0
                fb[i].temperature = -1; // 如果读取温度失败，设置为-1表示无效
            }
        }
        else
        {
            fb[i].position = 0.0f;
            fb[i].current = 0.0f;
            fb[i].temperature = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 避免过快循环
    }
    return success;
}
