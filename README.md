# BRIZO Firmware

BRIZO 机器鱼控制固件，运行于 ESP32-S3，基于 ESP-IDF 和 FreeRTOS。固件负责三关节 CAN 舵机控制、CPG 步态生成、CRSF 遥控输入、IMU901 数据采集，以及基于 UDP 的上下行通信。

## 功能概览

- 三个 Kingmax BLS4510S 舵机的 TWAI/CAN 控制与状态读取
- 50 Hz CPG、AHC 航向控制和预设动作重放
- CRSF 10 通道遥控输入与控制模式切换
- IMU901 姿态、加速度和角速度采集
- 多 WiFi 配置顺序连接和 UDP 双向通信
- 带帧头、消息 ID、长度和校验和的上下行二进制协议
- UDP 和 CRSF 输入超时保护

## 软件架构

```text
main/main.c
  |-- 初始化 LED、ADC、TWAI、CRSF、IMU、WiFi/UDP
  `-- robot_app_start()
        |-- cpg_control_task   控制源选择、CPG 解算、舵机指令
        |-- imu_crsf_task      遥控器和 IMU 数据汇总
        |-- telemetry_task     CAN 反馈采集、上行遥测打包
        `-- udp_rx_task        下行协议解析和命令更新

main/app/          应用逻辑、CPG、遥控映射
main/drivers/      硬件和通信驱动
main/protocols/    downward 解析与 upward 打包
```

`main.c` 只负责系统启动。FreeRTOS 应用任务集中在 `main/app/robot_app.c`，驱动内部还会创建 CRSF UART 接收任务和 IMU UART 事件任务。

## 硬件配置

| 功能 | ESP32-S3 资源 | 参数 |
| --- | --- | --- |
| TWAI TX | GPIO 4 | 500 kbit/s |
| TWAI RX | GPIO 5 | 500 kbit/s |
| IMU TX | GPIO 6, UART1 | 115200, 8N1 |
| IMU RX | GPIO 7, UART1 | 115200, 8N1 |
| CRSF RX | GPIO 2, UART2 | 420000, 8N1 |
| CRSF TX | GPIO 37, UART2 | 420000, 8N1 |
| ADC | GPIO 16 | 12 bit, 12 dB attenuation |
| 状态 LED | GPIO 17 | 低电平点亮 |

舵机节点 ID 为 `0x25`、`0x26`、`0x27`。启动后固件会先向三个舵机发送零位指令。

## FreeRTOS 任务

| 任务 | 周期/阻塞方式 | 优先级 | 核心 | 职责 |
| --- | --- | ---: | ---: | --- |
| `cpg_ctrl` | 20 ms | 5 | 1 | 控制源选择、CPG 更新、舵机位置发送 |
| `imu_crsf` | 10 ms | 4 | 1 | 获取 CRSF、姿态和惯性数据 |
| `telemetry` | 20 ms | 3 | 0 | 分时查询舵机反馈并发送遥测 |
| `udp_rx` | UDP 接收，最长阻塞 100 ms | 2 | 0 | 校验并保存下行关节指令 |

遥测任务每个周期只执行一次 CAN 查询，并在位置和电流之间交替，以减少控制任务受到 CAN 阻塞的影响。完整轮询三个舵机的一项反馈约需 120 ms，UDP 遥测包仍按 50 Hz 发送最近一次有效值。

## 控制模式

CRSF 原始通道值会先限制到 `1000..2000`，然后按旧版固件逻辑转换为 `3000 - value`。

| 通道 | 条件 | 行为 |
| --- | --- | --- |
| CH7 | `> 1500` | 使用 UDP 下行关节角控制，默认模式 |
| CH7 | `<= 1500` | 使用内部 CPG/AHC/动作重放 |
| CH5 | `> 1750` | 关节回中，清零 CPG 幅值和偏置状态 |
| CH5 | `1250 < value <= 1750` | 手动 CPG，无 AHC |
| CH5 | `<= 1250` | CPG + AHC 航向闭环 |
| CH9 | `< 1250` | 触发反向动作重放 |
| CH10 | `< 1250` | 触发正向动作重放 |

关节目标角限制为 `[-1.047, 1.047] rad`，约等于正负 60 度。UDP 或 CRSF 连续 500 ms 没有新数据时，相应控制输出会进入零位保护，避免持续使用过期指令。

## 网络配置

UDP 默认使用以下端口：

- 本地监听端口：`2333`
- 上位机目标端口：`6060`

WiFi 和目标 IP 配置位于 `main/main.c` 的 `udp_configs` 数组。固件按数组顺序尝试连接，成功后创建一个 UDP socket，同时用于接收控制指令和发送遥测。

SSID、密码和目标 IP 当前直接写在源代码中。部署到其他环境前应修改配置；正式发布时建议改为 menuconfig、NVS 或受控的本地配置文件，避免提交真实凭据。

## 通信协议

所有多字节数值当前均按 ESP32-S3 的小端序传输。`float` 为 IEEE 754 单精度。校验和为 `Msg ID + Payload Length + Payload` 所有字节累加后的低 8 位。

### 下行关节控制

| 字段 | 长度 | 内容 |
| --- | ---: | --- |
| Header | 2 bytes | `0xBB 0x66` |
| Msg ID | 1 byte | `0x10` |
| Payload Length | 1 byte | `28` |
| `target_theta[3]` | 12 bytes | 目标角度，单位 rad |
| `target_speed[3]` | 12 bytes | 目标速度，单位 rad/s |
| `timestamp_ms` | 4 bytes | 上位机时间戳 |
| Checksum | 1 byte | 8 bit 累加和 |

完整下行帧长度为 33 bytes。目前 `target_speed` 和 `timestamp_ms` 会参与格式和有限值校验，但控制器只使用 `target_theta`。

Python 打包示例：

```python
import struct

theta = [0.0, 0.2, -0.2]
speed = [0.0, 0.0, 0.0]
timestamp_ms = 0

payload = struct.pack("<6fI", *(theta + speed), timestamp_ms)
frame = bytearray((0xBB, 0x66, 0x10, len(payload)))
frame.extend(payload)
frame.append(sum(frame[2:]) & 0xFF)
```

### 上行综合遥测

| 字段 | 长度 | 内容 |
| --- | ---: | --- |
| Header | 2 bytes | `0xAA 0x55` |
| Msg ID | 1 byte | `0x01` |
| Payload Length | 1 byte | `84` |
| `id[3]` | 12 bytes | 三个 `int32_t` 舵机 ID |
| `theta[3]` | 12 bytes | 三个 `float` 关节角，单位 rad |
| `vin[3]` | 12 bytes | 三个 `float` 舵机电流，单位 A |
| `temp[3]` | 12 bytes | 三个 `int32_t` ADC 换算温度 |
| `rpy[3]` | 12 bytes | Roll/Pitch/Yaw，单位 deg |
| `acc[3]` | 12 bytes | 加速度，单位 g |
| `gyro[3]` | 12 bytes | 角速度，单位 deg/s |
| Checksum | 1 byte | 8 bit 累加和 |

完整上行帧长度为 89 bytes。上位机可使用格式字符串 `<3i3f3f3i3f3f3f` 解包 84-byte payload。

## 与旧版 `.ino` 的兼容性

控制算法、引脚、WiFi 列表、UDP 端口、CRSF 映射、舵机 ID、CPG 参数和重放轨迹均参考旧版 Arduino 固件迁移，但网络报文不再兼容旧版裸结构体格式：

- 旧版下行直接发送 24-byte `theta + speed`；当前要求 33-byte 完整协议帧。
- 旧版上行直接发送 84-byte 遥测结构体；当前发送 89-byte 带帧头和校验的协议帧。
- 当前增加了 UDP/CRSF 超时归零、非有限浮点数检查和关节限幅。
- 当前未实现旧版 `ArduinoOTA` 功能。
- 当前尚未实现旧版主循环中的 TWAI bus-off 自动重装逻辑。

因此，继续使用旧上位机程序时，必须同步升级其收发协议。

## 构建与烧录

要求：

- ESP-IDF 5.x，当前工程目标为 ESP32-S3
- 支持 ESP32-S3 的 USB 串口驱动和数据线
- 4 MB Flash 配置

先加载 ESP-IDF 环境，再在仓库根目录执行：

```bash
. /path/to/esp-idf/export.sh
idf.py build
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```

Linux 串口通常为 `/dev/ttyUSB0` 或 `/dev/ttyACM0`。退出串口监视器使用 `Ctrl+]`。

首次切换芯片目标时可执行：

```bash
idf.py set-target esp32s3
idf.py build
```

## 联调检查

1. 上电后确认三个舵机收到零位指令，串口日志没有 TWAI 初始化错误。
2. 确认日志显示目标 WiFi 已连接、本地 IP 已获取、UDP socket 已绑定到端口 `2333`。
3. 发送一帧合法的 `0x10` 下行数据，并确认 CH7 位于 UDP 控制档。
4. 检查舵机角度是否受到正负 60 度限制；停止发送 500 ms 后应回到零位。
5. 在上位机检查 89-byte 上行帧的帧头、长度和校验和，再解析遥测 payload。
6. 切换 CH7、CH5、CH9 和 CH10，分别验证 UDP、CPG、AHC 和动作重放。

## 已知限制

- WiFi 配置和凭据尚未从应用代码中分离。
- UDP 协议直接复制 packed C 结构体，默认依赖小端 IEEE 754 平台；跨平台实现应显式编码每个字段。
- `target_speed` 和下行时间戳尚未参与执行控制。
- OTA、TWAI bus-off 自动恢复和运行时 WiFi 配置尚未迁移。
- 最终发布前仍需在实机上验证舵机方向、机械零位、ADC 温度换算和失控保护。
