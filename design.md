# Exo Motor Control - 三层架构设计文档

## 1. 项目概述

外骨骼机器人双电机（左/右）力矩控制系统，采用三层解耦架构：

```
用户程序 (Python)          ← 第三层：应用逻辑
    │  TCP JSON (localhost:9090)
    ▼
motor_server (Python)      ← 第二层：PC 端驱动/代理
    │  USB Serial (packed struct 二进制帧)
    ▼
exo_controller (Teensy 4.1) ← 第一层：嵌入式固件，CAN 电机控制
    │  CAN Bus (1Mbps, MIT mini cheetah 协议)
    ▼
左电机 (ID=65)  /  右电机 (ID=33)
```

**设计理念**：类似 Linux 驱动架构（open/close/read/write），将通信细节（CRC、帧同步、心跳、重传）封装在中间层，用户只需调用简洁的 Python API。

## 2. 硬件配置

| 组件 | 型号/规格 |
|------|----------|
| MCU | Teensy 4.1 (ARM Cortex-M7, 600MHz) |
| 电机协议 | MIT mini cheetah CAN protocol |
| CAN 总线 | CAN1, 1Mbps |
| 左电机 CAN ID | 65 |
| 右电机 CAN ID | 33 |
| USB | Teensy 原生 USB 480Mbps（baud rate 参数无效） |
| LED | RGB 三色 (R=Pin14, G=Pin25, B=Pin24, 低电平有效) |

**电机参数**（`MotorParams` 命名空间）：
- P_MAX = 12.5 rad
- V_MAX = 50.0 rad/s
- I_MAX = 25.0 A
- KT = 0.091 Nm/A（力矩常数）

## 3. 核心设计决策

### 3.1 命令即心跳

motor_server 以 **200Hz** 持续向 Teensy 发送 ControlFrame，帧本身就是心跳。无需单独的心跳包。Teensy 若 **200ms** 内未收到任何 ControlFrame，自动触发安全停机。

### 3.2 仅力矩模式

系统只支持力矩控制（pure torque mode）。CAN 指令中 p_des=0, v_des=0, kp=0, kd=0，仅使用 i_ff（前馈电流）。

### 3.3 力矩安全限制（三重防护）

1. **motor_server 全局硬限制**：`TORQUE_HARD_LIMIT = 1.0 Nm`（文件顶部全局变量，根据电机型号修改）
   - `set_torque` 命令收到超限力矩 → 钳位 + 红色报警日志
   - `set_torque_limit` 命令也不能超过硬限制
   - TX 线程发送 ControlFrame 前再做一次钳位（最后防线）
2. **Teensy 固件限制**：`constrain(torque, -torque_limit, torque_limit)`
3. **CAN 电机自身限制**：电机驱动器内部电流限制

### 3.4 安全链（优先级从高到低）

| 优先级 | 机制 | 响应时间 |
|--------|------|----------|
| 1 | Teensy 硬件超时 (200ms 无帧 → SAFE_STOP) | ~200ms |
| 2 | motor_server 检测用户 TCP 断连 → 立即发 DISABLE | ~5ms |
| 3 | motor_server SIGINT/SIGTERM → 发 DISABLE 再退出 | ~10ms |
| 4 | PC 断电 → Teensy 200ms 超时自动触发 | ~200ms |

**关键原则**：任何一层断连都能安全停机，不可被上层软件绕过。

## 4. USB Serial 帧协议

所有帧使用 **packed struct**，小端字节序，CRC-8-CCITT（多项式 0x07）校验。

### 4.1 帧类型总览

| 帧头 | 方向 | 类型 | 大小 | 频率 |
|------|------|------|------|------|
| 0xAA | PC → Teensy | ControlFrame | 17B | 200Hz 持续 |
| 0xBB | PC → Teensy | ActionFrame | 7B | 按需 |
| 0xCC | Teensy → PC | StatusFrame | 44B | 500Hz 持续 |
| 0xDD | Teensy → PC | AckFrame | 8B | 响应 ActionFrame |

### 4.2 ControlFrame (17 bytes, 0xAA)

PC → Teensy，200Hz 持续发送，兼作心跳。

```
[0xAA] [seq:u16] [left_torque:f32] [right_torque:f32] [torque_limit:f32] [crc8] [0x55]
  1B      2B           4B                4B                 4B             1B     1B
```

| 字段 | 类型 | 说明 |
|------|------|------|
| head | u8 | 固定 0xAA |
| seq | u16 | 递增序列号 |
| left_torque | f32 | 左电机目标力矩 (Nm) |
| right_torque | f32 | 右电机目标力矩 (Nm) |
| torque_limit | f32 | 力矩限幅 (Nm) |
| crc8 | u8 | CRC-8 (覆盖 seq ~ torque_limit) |
| tail | u8 | 固定 0x55 |

### 4.3 ActionFrame (7 bytes, 0xBB)

PC → Teensy，按需发送（ENABLE/DISABLE/ZERO/STOP/PING）。

```
[0xBB] [seq:u16] [action:u8] [target:u8] [crc8] [0x55]
  1B      2B        1B          1B         1B     1B
```

| action 值 | 含义 |
|-----------|------|
| 0x01 | ENABLE - 使能电机 |
| 0x02 | DISABLE - 禁用电机 |
| 0x03 | ZERO - 编码器置零 |
| 0x04 | STOP - 紧急停止 |
| 0x10 | PING - 连接探测 |

| target 值 | 含义 |
|-----------|------|
| 0x00 | 左电机 |
| 0x01 | 右电机 |
| 0x02 | 双电机 |

### 4.4 StatusFrame (44 bytes, 0xCC)

Teensy → PC，500Hz 持续发送（所有状态下都发送）。

```
[0xCC] [seq:u16] [state:u8] [error:u8] [l_pos:f32] [l_vel:f32] [l_cur:f32]
       [r_pos:f32] [r_vel:f32] [r_cur:f32] [l_torque_cmd:f32] [r_torque_cmd:f32]
       [time_us:u32] [motor_status:u8] [crc8] [0x55]
```

| 字段 | 说明 |
|------|------|
| seq | 回显最后 ControlFrame 的 seq |
| state | Teensy 状态机 (0=IDLE, 1=ARMED, 2=RUNNING, 3=SAFE_STOP) |
| error | 错误位掩码 (bit0=左超时, bit1=右超时, bit2=心跳丢失, bit3=CRC失败) |
| l_pos/l_vel/l_cur | 左电机位置(rad)/速度(rad/s)/电流(A) |
| r_pos/r_vel/r_cur | 右电机同上 |
| l/r_torque_cmd | 实际执行的力矩命令 (Nm) |
| time_us | Teensy 内部微秒时间戳 |
| motor_status | bit0=左使能, bit1=右使能, bit2=左连接, bit3=右连接 |

### 4.5 AckFrame (8 bytes, 0xDD)

Teensy → PC，响应每个 ActionFrame。

```
[0xDD] [seq:u16] [action:u8] [result:u8] [state:u8] [crc8] [0x55]
```

| result 值 | 含义 |
|-----------|------|
| 0x00 | OK |
| 0x01 | INVALID_STATE - 当前状态不允许此操作 |
| 0x02 | ERROR |

### 4.6 CRC-8 计算

多项式 0x07 (CRC-8-CCITT)，覆盖范围：帧头和尾之间的所有字段（不含 head、crc8、tail 本身）。

```python
def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc
```

## 5. Teensy 固件 (exo_controller.ino)

### 5.1 状态机

```
IDLE ──(ActionFrame ENABLE)──→ ARMED ──(首个有效 ControlFrame)──→ RUNNING
  ↑                                                                  │
  └── SAFE_STOP ←──(200ms超时 / DISABLE / STOP)─────────────────────┘
      (力矩归零, CAN disable, 500ms后自动回 IDLE)
```

| 状态 | LED | 说明 |
|------|-----|------|
| IDLE | 蓝色常亮 | 待机，等待 ENABLE |
| ARMED | 绿色慢闪(1Hz) | 电机已使能，等待首个 ControlFrame |
| RUNNING | 绿色呼吸 | 正在执行力矩控制 |
| SAFE_STOP | 红色快闪(5Hz) | 安全停机中，500ms 后回 IDLE |

### 5.2 架构

- **`setup()`**：初始化 Serial、CAN1(1Mbps)、电机状态、LED、IntervalTimer(500Hz)
- **`loop()`**：非阻塞处理 Serial 输入（按帧头 0xAA/0xBB 分发）、超时检测(200ms)、SAFE_STOP→IDLE 转换、LED 更新(20ms)
- **`controlISR()` (500Hz)**：仅在 RUNNING 下执行 CAN 通信（发送力矩指令+读取反馈），所有状态下都发送 StatusFrame

### 5.3 ISR 安全

`loop()` 写入共享控制数据时使用 `noInterrupts()`/`interrupts()` 保护，避免 ISR 读到半更新的数据。

```cpp
volatile struct {
    float    left_torque;
    float    right_torque;
    float    torque_limit;
    uint16_t seq;
    bool     updated;
} sharedControl;
```

### 5.4 CAN 电机通信

使用 MIT mini cheetah 协议：
- **Enable**: `[0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFC]`
- **Disable**: `[0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFD]`
- **Zero**: `[0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFE]`
- **Command**: 8 字节打包 (p_des, v_des, kp, kd, i_ff)，纯力矩模式下 p=v=kp=kd=0
- **Response**: 电机返回 ID + position + velocity + current

## 6. motor_server (motor_server.py)

### 6.1 状态机

```
DISCONNECTED ──(打开串口)──→ CONNECTED ──(用户 enable)──→ ACTIVE
                                ↑                            │
                                └── STOPPING ←──(disable)────┘
                                    (发 DISABLE, 等 ACK)
```

### 6.2 线程架构

| 线程 | 频率 | 职责 |
|------|------|------|
| Serial TX | 200Hz | 构建 ControlFrame 发送 + 发送 pending ActionFrame |
| Serial RX | 持续 | 解析 StatusFrame/AckFrame，更新 SharedState |
| Heartbeat Monitor | 0.5Hz | 每2秒打印心跳状态（链路存活/死亡、TX/RX帧率） |
| Main (TCP Server) | 事件驱动 | accept 连接，处理 JSON 命令 |

### 6.3 SharedState

线程间共享，用 `threading.Lock` 保护：
- 控制参数：left_torque, right_torque, torque_limit
- Teensy 反馈：MotorFeedback（位置/速度/电流/状态/错误/时间戳）
- 待发动作队列：pending_actions
- 帧计数器：tx_frame_count, rx_frame_count, rx_crc_fail_count

### 6.4 全局力矩硬限制

```python
TORQUE_HARD_LIMIT = 1.0  # 绝对最大力矩 (Nm)，根据电机型号修改
```

- `set_torque` 命令：超限 → 钳位到 ±TORQUE_HARD_LIMIT + 红色报警日志
- `set_torque_limit` 命令：不允许超过 TORQUE_HARD_LIMIT
- TX 线程发送前：再次 clamp（最后防线）

### 6.5 彩色日志系统

带方向标识的彩色日志，一目了然区分通信方向：

| 标识 | 颜色 | 含义 |
|------|------|------|
| `[→ MCU]` | 青色 | server → Teensy |
| `[← MCU]` | 品红 | Teensy → server |
| `[← USR]` | 绿色 | 用户程序 → server |
| `[→ USR]` | 黄色 | server → 用户程序 |
| `[  SYS]` | 白色 | 系统生命周期事件 |
| `[HEART]` | 蓝色 | 心跳监控状态行 |
| `[ ERR ]` | 红色 | 错误/警告 |

心跳监控每 2 秒输出一行：
```
[HEART] Link:ALIVE (3ms ago) | TX:200Hz RX:500Hz | Server:ACTIVE Teensy:RUNNING
```

### 6.6 TCP JSON 命令

| 命令 | 参数 | 说明 |
|------|------|------|
| `connect` | - | 探测 Teensy 连接（发 PING 等 ACK） |
| `enable` | target | 使能电机 |
| `disable` | target | 禁用电机 |
| `set_torque` | left, right | 设置力矩 (Nm)，受硬限制钳位 |
| `set_torque_limit` | limit | 设置力矩限幅 |
| `get_status` | - | 获取完整系统状态 |
| `zero` | target | 编码器置零 |
| `stop` | - | 紧急停止 |
| `disconnect` | - | 优雅断开 |

target 取值：`'left'`, `'right'`, `'both'`

通信格式：换行分隔的 JSON，如 `{"cmd": "set_torque", "left": 0.5, "right": -0.3}\n`

### 6.7 Windows 定时精度

使用 `ctypes.windll.winmm.timeBeginPeriod(1)` 提升 sleep 精度到 1ms，确保 200Hz TX 循环可达。

### 6.8 启动命令

```bash
python motor_server.py --port COM12 --tcp-port 9090 --log-level INFO
```

## 7. motor_client (motor_client.py)

### 7.1 API

```python
class MotorClient:
    def __init__(host='127.0.0.1', port=9090, timeout=5.0)

    # 连接管理
    def connect() -> dict
    def disconnect()
    # 支持 with 语句: __enter__ 调用 connect, __exit__ 调用 disable + disconnect

    # 电机控制
    def enable(target='both') -> dict
    def disable(target='both') -> dict
    def set_torque(left=0.0, right=0.0) -> dict
    def set_torque_limit(limit: float) -> dict
    def zero(target='both') -> dict
    def stop() -> dict
    def get_status() -> dict
```

### 7.2 用法示例

```python
from motor_client import MotorClient

with MotorClient() as mc:
    mc.set_torque_limit(1.0)
    mc.enable()
    mc.set_torque(left=0.5, right=-0.5)
    status = mc.get_status()
    print(status['data']['left']['current'])
    mc.stop()
    mc.disable()
```

### 7.3 内置测试模式

```bash
python motor_client.py test1   # 左右同向正弦力矩 (1Nm, 1Hz, 5秒)
python motor_client.py test2   # 左右反向正弦力矩 (1Nm, 1Hz, 5秒)
```

测试参数（文件内顶部可调）：
- `AMPLITUDE = 1.0` Nm
- `FREQUENCY = 1.0` Hz
- `DURATION = 5.0` s
- `LOOP_HZ = 100`（用户循环频率）

## 8. 文件清单

```
备份/
├── exo_controller.ino   ← Teensy 4.1 固件 (~743行)
├── motor_server.py      ← PC 端驱动/代理层 (~763行)
├── motor_client.py      ← 用户端客户端库 + 测试 (~332行)
└── design.md            ← 本文档
```

## 9. 快速上手

```bash
# 1. 编译并烧录 exo_controller.ino 到 Teensy 4.1 (Arduino IDE / PlatformIO)

# 2. 安装 Python 依赖
pip install pyserial

# 3. 启动 motor_server（替换为实际串口号）
python motor_server.py --port COM12

# 4. 运行测试
python motor_client.py test1
```

## 10. 扩展指南

### 添加新测试模式

在 `motor_client.py` 的 `__main__` 部分：

1. 在 `TESTS` 字典添加条目：`'test3': '描述'`
2. 编写力矩生成函数：`def gen_torque_test3(t): return left, right`
3. 在 `gen_func` 字典添加映射

### 修改力矩限制

编辑 `motor_server.py` 第 70 行：
```python
TORQUE_HARD_LIMIT = 1.0  # 修改此值
```

### 修改电机 CAN ID

编辑 `exo_controller.ino` 第 14-15 行：
```cpp
#define LEFT_MOTOR_ID   65   // 修改此值
#define RIGHT_MOTOR_ID  33   // 修改此值
```
