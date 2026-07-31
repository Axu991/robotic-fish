/**
 * @file upward_packer.c
 * @brief 机器鱼上行协议打包器实现
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-31
 * @version 1.1
 */

#include "upward_packer.h"
#include <string.h>

/**
 * @brief 计算校验和 (Sum Check)
 * @note 校验范围涵盖：Msg ID + Payload Length + Payload 数据本身
 */
static uint8_t calculate_checksum(uint8_t msg_id, uint8_t len, const uint8_t *payload)
{
    uint8_t sum = 0;
    sum += msg_id;
    sum += len;
    for (uint8_t i = 0; i < len; i++) {
        sum += payload[i];
    }
    return sum;
}

bool upward_pack_custom(uint8_t msg_id, const uint8_t *payload, uint8_t payload_len, uint8_t *out_buf, size_t *out_len)
{
    if (out_buf == NULL || out_len == NULL) {
        return false;
    }
    if (payload_len > UPWARD_MAX_PAYLOAD_LEN
            || (payload_len > 0 && payload == NULL)) {
        return false; // 数据超出缓冲区界限
    }

    uint8_t idx = 0;

    // 1. 填充 2 字节包头 (0xAA 0x55)
    out_buf[idx++] = UPWARD_FRAME_HEADER1;
    out_buf[idx++] = UPWARD_FRAME_HEADER2;

    // 2. 填充 1 字节 Message ID 与 1 字节 Payload Length
    out_buf[idx++] = msg_id;
    out_buf[idx++] = payload_len;

    // 3. 复制数据载荷
    if (payload_len > 0 && payload != NULL) {
        memcpy(&out_buf[idx], payload, payload_len);
        idx += payload_len;
    }

    // 4. 计算并附加 1 字节校验和
    uint8_t checksum = calculate_checksum(msg_id, payload_len, payload);
    out_buf[idx++] = checksum;

    // 返回总字节数
    *out_len = idx;
    return true;
}

bool upward_pack_telemetry(const upward_payload_telemetry_t *payload, uint8_t *out_buf, size_t *out_len)
{
    if (payload == NULL) {
        return false;
    }
    return upward_pack_custom(UPWARD_MSG_TELEMETRY, 
                              (const uint8_t *)payload, 
                              sizeof(upward_payload_telemetry_t), 
                              out_buf, 
                              out_len);
}
