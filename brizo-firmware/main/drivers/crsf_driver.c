/**
 * @file crsf_driver.c
 * @brief CRSF 协议驱动实现文件
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-31
 * @version 1.0
 */

#include "crsf_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>



static const char *TAG = "CRSF_DRIVER";

// 静态变量
static uart_port_t s_uart_num = UART_NUM_2;
static crsf_channels_t s_channels = {
    .ch = {1500, 1500, 2000, 1500, 2000, 2000, 2000, 2000, 2000, 2000},
};
static bool s_data_ready = false;
static SemaphoreHandle_t s_mutex = NULL;


// ---------- CRSF 协议常量 ----------
#define CRSF_SYNC_BYTE      0xC8
#define CRSF_FRAME_TYPE_RC  0x16   // RC channels frame
#define CRSF_MAX_PACKET_LEN 64

// CRC 表（多项式 0xD5，初始 0x00）
static const uint8_t crsf_crc8_table[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
    0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
    0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
    0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
    0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
    0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
    0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
    0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
    0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
    0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

static uint8_t crsf_crc8(uint8_t crc, uint8_t value)
{
    return crsf_crc8_table[crc ^ value];
}

// ---------- 解析状态机 ----------
typedef enum {
    CRSF_STATE_WAIT_SYNC,
    CRSF_STATE_WAIT_LENGTH,
    CRSF_STATE_READ_BODY,
} crsf_state_t;

static crsf_state_t s_state = CRSF_STATE_WAIT_SYNC;
static uint8_t s_frame_body[CRSF_MAX_PACKET_LEN];
static uint8_t s_body_index = 0;
static uint8_t s_packet_len = 0;

static uint16_t crsf_raw_to_us(uint16_t raw)
{
    const uint16_t raw_min = 172;
    const uint16_t raw_max = 1811;
    if (raw < raw_min) raw = raw_min;
    if (raw > raw_max) raw = raw_max;
    return 1000U + (uint16_t)(((uint32_t)(raw - raw_min) * 1000U) / (raw_max - raw_min));
}

static void parse_rc_channels(const uint8_t *payload, size_t payload_len)
{
    if (payload_len < 22) return;

    crsf_channels_t parsed;
    for (int i = 0; i < CRSF_CHANNEL_COUNT; i++) {
        const int bit = i * 11;
        const int byte_idx = bit / 8;
        const int bit_off = bit % 8;
        uint32_t packed = (uint32_t)payload[byte_idx]
                        | ((uint32_t)payload[byte_idx + 1] << 8)
                        | ((uint32_t)payload[byte_idx + 2] << 16);
        parsed.ch[i] = crsf_raw_to_us((packed >> bit_off) & 0x7FFU);
    }

    if (s_mutex != NULL) xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_channels = parsed;
    s_data_ready = true;
    if (s_mutex != NULL) xSemaphoreGive(s_mutex);
}


// ---------- 解析函数 ----------
static void parse_crsf_byte(uint8_t byte)
{
    switch (s_state) {
        case CRSF_STATE_WAIT_SYNC:
            if (byte == CRSF_SYNC_BYTE) {
                s_state = CRSF_STATE_WAIT_LENGTH;
            }
            break;

        case CRSF_STATE_WAIT_LENGTH:
            s_packet_len = byte;
            s_body_index = 0;
            if (s_packet_len < 2 || s_packet_len > CRSF_MAX_PACKET_LEN) {
                s_state = CRSF_STATE_WAIT_SYNC;
            } else {
                s_state = CRSF_STATE_READ_BODY;
            }
            break;

        case CRSF_STATE_READ_BODY:
            s_frame_body[s_body_index++] = byte;
            if (s_body_index == s_packet_len) {
                uint8_t crc = 0;
                for (uint8_t i = 0; i < s_packet_len - 1; i++) {
                    crc = crsf_crc8(crc, s_frame_body[i]);
                }
                if (crc == s_frame_body[s_packet_len - 1]
                        && s_frame_body[0] == CRSF_FRAME_TYPE_RC) {
                    parse_rc_channels(&s_frame_body[1], s_packet_len - 2);
                }
                s_state = CRSF_STATE_WAIT_SYNC;
            }
            break;

        default:
            s_state = CRSF_STATE_WAIT_SYNC;
            break;
    }
}

// ---------- 任务函数 ----------
static void crsf_task(void *pvParameters)
{
    uint8_t byte;
    while (1) {
        if (uart_read_bytes(s_uart_num, &byte, 1, pdMS_TO_TICKS(10)) > 0) {
            parse_crsf_byte(byte);
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

// ---------- 公开接口 ----------
void crsf_init(uart_port_t uart_num, gpio_num_t rx_pin, gpio_num_t tx_pin)
{
    s_uart_num = uart_num;

    // 配置 UART
    uart_config_t uart_config = {
        .baud_rate = CRSF_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, 256, 0, 0, NULL, 0));

    // 创建互斥量
    s_mutex = xSemaphoreCreateMutex();

    // 创建解析任务
    xTaskCreate(crsf_task, "crsf_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "CRSF driver initialized on UART%d", uart_num);
}

bool crsf_get_channels(crsf_channels_t *channels)
{
    if (channels == NULL) return false;
    if (s_mutex != NULL) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
    }
    bool ready = s_data_ready;
    memcpy(channels, &s_channels, sizeof(crsf_channels_t));
    s_data_ready = false;
    if (s_mutex != NULL) {
        xSemaphoreGive(s_mutex);
    }
    return ready;
}

uint16_t crsf_get_channel(uint8_t index)
{
    if (index >= CRSF_CHANNEL_COUNT) return 1500;
    uint16_t val = 1500;
    if (s_mutex != NULL) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
    }
    val = s_channels.ch[index];
    if (s_mutex != NULL) {
        xSemaphoreGive(s_mutex);
    }
    return val;
}
