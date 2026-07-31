/**
 * @file downward_parser.c
 * @brief 机器鱼下行控制协议解析器实现
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-31
 * @version 1.0
 */

#include "downward_parser.h"
#include <string.h>

void downward_parser_init(downward_parser_t *parser)
{
    if (parser == NULL) return;
    parser->state = STATE_WAIT_HEADER1;
    parser->rx_idx = 0;
    parser->calculated_checksum = 0;
}

bool downward_parser_feed_byte(downward_parser_t *parser, uint8_t byte, downward_frame_t *out_frame)
{
    if (parser == NULL || out_frame == NULL) return false;

    switch (parser->state) {
        case STATE_WAIT_HEADER1:
            if (byte == DOWNWARD_FRAME_HEADER1) {
                parser->state = STATE_WAIT_HEADER2;
            }
            break;

        case STATE_WAIT_HEADER2:
            if (byte == DOWNWARD_FRAME_HEADER2) {
                parser->state = STATE_WAIT_MSG_ID;
            } else {
                parser->state = STATE_WAIT_HEADER1; // 重新等待 Header1
            }
            break;

        case STATE_WAIT_MSG_ID:
            parser->rx_msg_id = byte;
            parser->calculated_checksum = byte;
            parser->state = STATE_WAIT_LEN;
            break;

        case STATE_WAIT_LEN:
            parser->rx_payload_len = byte;
            parser->calculated_checksum += byte;
            if (parser->rx_payload_len > sizeof(parser->rx_buf)) {
                // 超长非法数据包，重置状态
                parser->state = STATE_WAIT_HEADER1;
            } else if (parser->rx_payload_len == 0) {
                parser->state = STATE_WAIT_CHECKSUM;
            } else {
                parser->rx_idx = 0;
                parser->state = STATE_WAIT_PAYLOAD;
            }
            break;

        case STATE_WAIT_PAYLOAD:
            parser->rx_buf[parser->rx_idx++] = byte;
            parser->calculated_checksum += byte;
            if (parser->rx_idx >= parser->rx_payload_len) {
                parser->state = STATE_WAIT_CHECKSUM;
            }
            break;

        case STATE_WAIT_CHECKSUM:
            parser->state = STATE_WAIT_HEADER1; // 重置状态备用
            if (byte == parser->calculated_checksum) {
                // 校验成功，提取数据输出
                out_frame->msg_id = parser->rx_msg_id;
                out_frame->payload_len = parser->rx_payload_len;
                memcpy(&out_frame->payload, parser->rx_buf, parser->rx_payload_len);
                return true;
            }
            break;

        default:
            parser->state = STATE_WAIT_HEADER1;
            break;
    }

    return false;
}

bool downward_parse_packet(const uint8_t *buf, size_t len, downward_frame_t *out_frame)
{
    if (buf == NULL || out_frame == NULL || len < 5) { // 包头(2) + ID(1) + Len(1) + Checksum(1)
        return false;
    }

    // 1. 检查帧头
    if (buf[0] != DOWNWARD_FRAME_HEADER1 || buf[1] != DOWNWARD_FRAME_HEADER2) {
        return false;
    }

    uint8_t msg_id = buf[2];
    uint8_t payload_len = buf[3];

    // 2. 检查长度匹配
    if (len != (size_t)(4 + payload_len + 1)) {
        return false;
    }

    // 3. 计算校验和
    uint8_t checksum = msg_id + payload_len;
    for (size_t i = 0; i < payload_len; i++) {
        checksum += buf[4 + i];
    }

    // 4. 比对校验和
    if (checksum != buf[4 + payload_len]) {
        return false;
    }

    // 5. 复制解包数据
    out_frame->msg_id = msg_id;
    out_frame->payload_len = payload_len;
    memcpy(&out_frame->payload, &buf[4], payload_len);

    return true;
}