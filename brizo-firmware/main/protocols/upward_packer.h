/**
 * @file upward_packer.h
 * @brief 机器鱼上行协议打包器
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-31
 * @version 1.1
 */

#ifndef UPWARD_PACKER_H
#define UPWARD_PACKER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 帧头与系统常数定义 */
#define UPWARD_FRAME_HEADER1   0xAA
#define UPWARD_FRAME_HEADER2   0x55
#define SERVO_NUMS             3

/* 消息类型 ID */
typedef enum {
    UPWARD_MSG_TELEMETRY   = 0x01,  /*!< 综合遥测帧 (舵机状态 + IMU + 系统参数) */
    UPWARD_MSG_IMU_ONLY    = 0x02,  /*!< 仅 IMU 姿态与高频惯性数据 */
    UPWARD_MSG_CUSTOM      = 0xFF,  /*!< 通用自定义二进制载荷 */
} upward_msg_id_t;

/**
 * @brief 映射机器鱼完整的上行数据帧 Payload 
 * @note 使用 __attribute__((packed)) 强制单字节对齐
 */
typedef struct __attribute__((packed)) {
    int32_t  id[SERVO_NUMS];     /*!< 舵机 CAN ID [0x25, 0x26, 0x27] (12 bytes) */
    float    theta[SERVO_NUMS];  /*!< 舵机相对中心点的当前角度 [rad] (12 bytes) */
    float    vin[SERVO_NUMS];    /*!< 舵机相电流 [A] (12 bytes) */
    int32_t  temp[SERVO_NUMS];   /*!< 预估/实测温度 [Celsius] (12 bytes) */
    float    rpy[3];             /*!< 姿态角 [Roll, Pitch, Yaw] [deg] (12 bytes) */
    float    acc[3];             /*!< 三轴加速度 [g] (12 bytes) */
    float    gyro[3];            /*!< 三轴角速度 [deg/s] (12 bytes) */
} upward_payload_telemetry_t;

/* 最大 Buffer 尺寸计算 */
#define UPWARD_MAX_PAYLOAD_LEN   sizeof(upward_payload_telemetry_t) // 84 字节
#define UPWARD_MAX_PACKET_LEN    (4 + UPWARD_MAX_PAYLOAD_LEN + 1)   // Header(2)+ID(1)+Len(1) + Payload + Checksum(1) = 89 字节

/**
 * @brief 打包机器鱼全量遥测数据 (Telemetry Packet)
 * 
 * @param payload 输入的遥测数据结构体指针
 * @param out_buf 输出字节缓冲区 (空间需 >= UPWARD_MAX_PACKET_LEN)
 * @param out_len 打包完成后实际的帧总长度
 * @return true 打包成功
 * @return false 参数非法或内存溢出
 */
bool upward_pack_telemetry(const upward_payload_telemetry_t *payload, uint8_t *out_buf, size_t *out_len);

/**
 * @brief 自定义通用数据打包函数
 * 
 * @param msg_id 消息 ID (参见 upward_msg_id_t)
 * @param payload 二进制载荷指针
 * @param payload_len 载荷长度
 * @param out_buf 输出字节缓冲区
 * @param out_len 打包完成后实际的帧总长度
 * @return true 打包成功
 * @return false 参数非法或长度超限
 */
bool upward_pack_custom(uint8_t msg_id, const uint8_t *payload, uint8_t payload_len, uint8_t *out_buf, size_t *out_len);

#ifdef __cplusplus
}
#endif

#endif // UPWARD_PACKER_H