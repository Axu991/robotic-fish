/**
 * @file downward_parser.h
 * @brief 机器鱼下行控制协议解析器 (ESP32-S3 ESP-IDF)
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-31
 * @version 1.0
 */

#ifndef DOWNWARD_PARSER_H
#define DOWNWARD_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 帧头定义 (与上位机约定的下行帧头) */
#define DOWNWARD_FRAME_HEADER1   0xBB
#define DOWNWARD_FRAME_HEADER2   0x66

#define DOWNWARD_SERVO_NUMS      3

/* 下行消息类型定义 */
typedef enum {
    DOWNWARD_MSG_JOINTS_CMD  = 0x10,  /*!< 关节直接角度控制指令 (Sim2Real / RL 透传) */
    DOWNWARD_MSG_CPG_PARAMS  = 0x11,  /*!< CPG 步态参数实时重构 (幅值, 频率, 偏置) */
    DOWNWARD_MSG_MODE_SWITCH = 0x12,  /*!< 模式切换指令 (Manual / CPG / RL / Replay) */
} downward_msg_id_t;

/**
 * @brief 关节角度直接控制 Payload (匹配下行 3 关节指令)
 */
typedef struct __attribute__((packed)) {
    float target_theta[DOWNWARD_SERVO_NUMS]; /*!< 目标关节角 [rad] (12 bytes) */
    float target_speed[DOWNWARD_SERVO_NUMS]; /*!< 目标关节速度 [rad/s] (12 bytes) */
    uint32_t timestamp_ms;                   /*!< 上位机控制时间戳 [ms] (4 bytes) */
} downward_payload_joints_cmd_t;

/**
 * @brief 解析完成后的统一数据帧结构
 */
typedef struct {
    uint8_t msg_id;
    uint8_t payload_len;
    union {
        downward_payload_joints_cmd_t joints_cmd;
        uint8_t raw_payload[64];
    } payload;
} downward_frame_t;

/**
 * @brief FSM 解析器句柄
 */
typedef struct {
    enum {
        STATE_WAIT_HEADER1,
        STATE_WAIT_HEADER2,
        STATE_WAIT_MSG_ID,
        STATE_WAIT_LEN,
        STATE_WAIT_PAYLOAD,
        STATE_WAIT_CHECKSUM
    } state;

    uint8_t rx_msg_id;
    uint8_t rx_payload_len;
    uint8_t rx_buf[64];
    uint8_t rx_idx;
    uint8_t calculated_checksum;
} downward_parser_t;

/**
 * @brief 初始化下行解析器句柄
 * 
 * @param parser 解析器实例指针
 */
void downward_parser_init(downward_parser_t *parser);

/**
 * @brief 逐字节输入解析 (流式/状态机解析，适合 UDP / UART)
 * 
 * @param parser 解析器实例指针
 * @param byte 输入的单个字节
 * @param out_frame 解析成功的完整数据帧存放地址
 * @return true 成功解析出一帧有效数据
 * @return false 还在接收或校验失败
 */
bool downward_parser_feed_byte(downward_parser_t *parser, uint8_t byte, downward_frame_t *out_frame);

/**
 * @brief 块数据解析 (直接解析整包 UDP 缓冲区)
 * 
 * @param buf UDP 接收到的原始字节数组
 * @param len 缓冲区字节长度
 * @param out_frame 解析成功的数据帧存放地址
 * @return true 解析成功且校验通过
 * @return false 包头错或校验失败
 */
bool downward_parse_packet(const uint8_t *buf, size_t len, downward_frame_t *out_frame);

#ifdef __cplusplus
}
#endif

#endif // DOWNWARD_PARSER_H