/**
 * @file imu901_driver.h
 * @brief IMU 驱动头文件, 适用于 ATK-IMU601,901 (ESP-IDF版)
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-30
 * @version 2.0
 */

#ifndef IMU901_DRIVER_H
#define IMU901_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------- 帧消息 ID 定义 ----------
#define UP_ATTITUDE     0x01
#define UP_QUAT         0x02
#define UP_GYROACCDATA  0x03
#define UP_MAGDATA      0x04
#define UP_BARODATA     0x05

// ---------- 寄存器地址定义 ----------
#define REG_GYROFSR     0x11
#define REG_ACCFSR      0x12
#define REG_UPSET       0x18
#define REG_UPRATE      0x19
#define REG_SAVE        0x00

// ---------- 暴露的数据结构体 ----------

/**
 * @brief 欧拉角 (姿态角)
 */
typedef struct {
    float roll;  ///< 横滚角 (°)
    float pitch; ///< 俯仰角 (°)
    float yaw;   ///< 航向角 (°)
} attitude_t;

/**
 * @brief 陀螺仪与加速度计数据
 */
typedef struct {
    int16_t acc[3];   ///< 原始加速度 RAW
    int16_t gyro[3];  ///< 原始角速度 RAW
    float faccG[3];   ///< 转换后的加速度 (g)
    float fgyroD[3];  ///< 转换后的角速度 (°/s)
} gyroAcc_t;

/**
 * @brief 四元数
 */
typedef struct {
    float q0, q1, q2, q3;
} quaternion_t;

/**
 * @brief 磁力计数据
 */
typedef struct {
    int16_t mag[3];  ///< 磁力计三轴 RAW
    float temp;      ///< 芯片温度 (°C)
} mag_t;

/**
 * @brief 气压计与高度数据
 */
typedef struct {
    int32_t pressure; ///< 气压 (Pa)
    int32_t altitude; ///< 相对高度 (cm)
    float temp;       ///< 温度 (°C)
} baro_t;

/**
 * @brief 寄存器配置参数
 */
typedef struct {
    uint8_t gyroFsr;
    uint8_t accFsr;
    uint8_t rate;
} regValue_t;

/**
 * @brief ATKP 传输协议帧结构体
 */
typedef struct {
    uint8_t startByte1;
    uint8_t startByte2;
    uint8_t msgID;
    uint8_t dataLen;
    uint8_t data[28];
    uint8_t checkSum;
} atkp_t;

// ---------- API 驱动接口 ----------

/**
 * @brief 初始化 IMU901 驱动并启动后台接收 Task
 * 
 * @param uart_num  使用的 UART 端口号 (如 UART_NUM_1)
 * @param tx_pin    ESP32 连接 IMU RX 的 GPIO
 * @param rx_pin    ESP32 连接 IMU TX 的 GPIO
 * @param baud_rate 串口波特率 (默认 115200)
 */
void imu_init(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, int baud_rate);

/**
 * @brief 暴露给外部调用的单字节解包函数 (供自定义串口或单元测试使用)
 * 
 * @param ch 输入字节
 * @param out_pkt 解包成功时输出的帧数据指针
 * @return true 校验成功且得到一帧完整数据
 * @return false 处于中间解包状态或校验失败
 */
bool imu901_unpack_byte(uint8_t ch, atkp_t *out_pkt);

// 数据获取接口 (线程安全，带更新标志校验)
bool imu_get_attitude(attitude_t *att);
bool imu_get_gyro_acc(gyroAcc_t *gyro_acc);
bool imu_get_quaternion(quaternion_t *quat);
bool imu_get_mag(mag_t *mag);
bool imu_get_baro(baro_t *baro);

#ifdef __cplusplus
}
#endif

#endif // IMU901_DRIVER_H