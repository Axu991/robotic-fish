#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"

// ESP32-S3 ROM 矩阵重映射头文件
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

#include "drivers/led_driver.h"
#include "drivers/imu901_driver.h"

#define LED_PIN        GPIO_NUM_17

#define IMU_UART_NUM   UART_NUM_1
#define IMU_TX_PIN     GPIO_NUM_6   // ESP32-S3 TX -> 接 IMU RX
#define IMU_RX_PIN     GPIO_NUM_7   // ESP32-S3 RX -> 接 IMU TX
#define IMU_BAUDRATE   115200

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "正在初始化系统...");

    // 1. 初始化 LED (若有驱动)
    // led_init(LED_PIN);

    // 2. 初始化 IMU901 (自动完成引脚映射与串口启动)
    imu_init(IMU_UART_NUM, IMU_TX_PIN, IMU_RX_PIN, IMU_BAUDRATE);

    // 数据接收结构体
    attitude_t attitude;
    gyroAcc_t gyro_acc;

    ESP_LOGI(TAG, "开始读取 IMU 数据...");

    while (1) {
        // 读取欧拉角 (Roll, Pitch, Yaw)
        if (imu_get_attitude(&attitude)) {
            ESP_LOGI(TAG, "[姿态角] Roll: %6.2f° | Pitch: %6.2f° | Yaw: %6.2f°",
                     attitude.roll, attitude.pitch, attitude.yaw);
        }

        // 读取加速度与角速度
        if (imu_get_gyro_acc(&gyro_acc)) {
            ESP_LOGI(TAG, "[加速度] X:%5.2fg  Y:%5.2fg  Z:%5.2fg | [陀螺仪] X:%6.1f°/s Y:%6.1f°/s Z:%6.1f°/s",
                     gyro_acc.faccG[0], gyro_acc.faccG[1], gyro_acc.faccG[2],
                     gyro_acc.fgyroD[0], gyro_acc.fgyroD[1], gyro_acc.fgyroD[2]);
        }

        // 每 100ms 刷新打印一次 (10Hz 频率)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}