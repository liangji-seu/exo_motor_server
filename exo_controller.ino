/*
 * exo_controller.ino - 三层架构 Teensy 固件
 *
 * Teensy 4.1 双CAN电机控制器
 * 与 motor_server (PC) 通过 USB Serial + packed struct 通信
 * 命令即心跳：每个 ControlFrame 都是心跳，无需单独心跳包
 *
 * 状态机: IDLE → ARMED → RUNNING → SAFE_STOP → IDLE
 */

#include <FlexCAN_T4.h>

// ==================== 配置参数 ====================
#define LEFT_MOTOR_ID   65
#define RIGHT_MOTOR_ID  33

#define CAN_BAUD        1000000

#define CONTROL_FREQ_HZ     500
#define CONTROL_PERIOD_US   (1000000 / CONTROL_FREQ_HZ)

#define HEARTBEAT_TIMEOUT_MS  200   // 200ms 无 ControlFrame 则安全停机
#define SAFE_STOP_GRACE_MS    500   // SAFE_STOP 状态持续时间

// ==================== LED 引脚配置 ====================
#define LED_R_PIN       14
#define LED_G_PIN       25
#define LED_B_PIN       24
#define LED_ACTIVE_LOW  true

// ==================== 电机参数 ====================
namespace MotorParams {
    constexpr float P_MAX = 12.5f;
    constexpr float V_MAX = 50.0f;
    constexpr float I_MAX = 25.0f;
    constexpr float KT = 0.091f;
    constexpr float TORQUE_LIMIT_DEFAULT = 5.0f;
}

// ==================== 帧协议常量 ====================
// 帧头字节
#define HEAD_CONTROL    0xAA  // PC → Teensy: ControlFrame
#define HEAD_ACTION     0xBB  // PC → Teensy: ActionFrame
#define HEAD_STATUS     0xCC  // Teensy → PC: StatusFrame
#define HEAD_ACK        0xDD  // Teensy → PC: AckFrame
#define FRAME_TAIL      0x55

// ActionFrame 动作类型
#define ACTION_ENABLE   0x01
#define ACTION_DISABLE  0x02
#define ACTION_ZERO     0x03
#define ACTION_STOP     0x04
#define ACTION_PING     0x10

// ActionFrame 目标
#define TARGET_LEFT     0x00
#define TARGET_RIGHT    0x01
#define TARGET_BOTH     0x02

// AckFrame 结果
#define RESULT_OK           0x00
#define RESULT_INVALID_STATE 0x01
#define RESULT_ERROR        0x02

// 错误码（位掩码）
#define ERR_NONE            0x00
#define ERR_LEFT_TIMEOUT    0x01
#define ERR_RIGHT_TIMEOUT   0x02
#define ERR_HEARTBEAT       0x04
#define ERR_CRC_FAIL        0x08

// ==================== 系统状态枚举 ====================
enum SystemState : uint8_t {
    STATE_IDLE      = 0,
    STATE_ARMED     = 1,
    STATE_RUNNING   = 2,
    STATE_SAFE_STOP = 3
};

// ==================== 通信帧结构（packed struct） ====================
#pragma pack(push, 1)

// PC → Teensy: 控制帧 (17 bytes)
struct ControlFrame {
    uint8_t  head;          // 0xAA
    uint16_t seq;
    float    left_torque;   // 力矩 (Nm)
    float    right_torque;
    float    torque_limit;
    uint8_t  crc8;
    uint8_t  tail;          // 0x55
};

// PC → Teensy: 动作帧 (7 bytes)
struct ActionFrame {
    uint8_t  head;          // 0xBB
    uint16_t seq;
    uint8_t  action;
    uint8_t  target;        // 0=left, 1=right, 2=both
    uint8_t  crc8;
    uint8_t  tail;          // 0x55
};

// Teensy → PC: 状态帧 (44 bytes)
struct StatusFrame {
    uint8_t  head;          // 0xCC
    uint16_t seq;           // echo last ControlFrame seq
    uint8_t  state;         // SystemState
    uint8_t  error_code;    // bitmask
    float    left_pos;
    float    left_vel;
    float    left_cur;
    float    right_pos;
    float    right_vel;
    float    right_cur;
    float    left_torque_cmd;
    float    right_torque_cmd;
    uint32_t teensy_time_us;
    uint8_t  motor_status;  // bit0=L_en, bit1=R_en, bit2=L_conn, bit3=R_conn
    uint8_t  crc8;
    uint8_t  tail;          // 0x55
};

// Teensy → PC: 应答帧 (8 bytes)
struct AckFrame {
    uint8_t  head;          // 0xDD
    uint16_t seq;           // echo ActionFrame seq
    uint8_t  action;
    uint8_t  result;        // 0=OK, 1=invalid_state, 2=error
    uint8_t  state;         // current state after action
    uint8_t  crc8;
    uint8_t  tail;          // 0x55
};

#pragma pack(pop)

// ==================== 电机状态结构 ====================
struct MotorState {
    uint8_t id;
    volatile bool enabled;
    volatile float position;
    volatile float velocity;
    volatile float current;
    float torque_cmd;
    int timeout_count;
    volatile bool connected;
};

// ==================== LED控制类 ====================
class StatusLed {
public:
    StatusLed(int r_pin, int g_pin, int b_pin, bool active_low = false) {
        _r_pin = r_pin;
        _g_pin = g_pin;
        _b_pin = b_pin;
        _active_low = active_low;

        pinMode(_r_pin, OUTPUT);
        pinMode(_g_pin, OUTPUT);
        pinMode(_b_pin, OUTPUT);

        setColor(0, 0, 0);
    }

    void setColor(uint8_t r, uint8_t g, uint8_t b) {
        if (_active_low) {
            analogWrite(_r_pin, 255 - r);
            analogWrite(_g_pin, 255 - g);
            analogWrite(_b_pin, 255 - b);
        } else {
            analogWrite(_r_pin, r);
            analogWrite(_g_pin, g);
            analogWrite(_b_pin, b);
        }
    }

    void update(SystemState state) {
        unsigned long now = millis();
        switch (state) {
            case STATE_IDLE:
                setColor(0, 0, 255);  // 蓝色常亮
                break;
            case STATE_ARMED: {
                // 绿色慢闪 (1Hz)
                bool on = ((now / 500) % 2) == 0;
                setColor(0, on ? 255 : 0, 0);
                break;
            }
            case STATE_RUNNING:
                pulse(0, 255, 0, 1000, now);  // 绿色呼吸
                break;
            case STATE_SAFE_STOP: {
                // 红色快闪 (5Hz)
                bool on2 = ((now / 100) % 2) == 0;
                setColor(on2 ? 255 : 0, 0, 0);
                break;
            }
        }
    }

private:
    int _r_pin, _g_pin, _b_pin;
    bool _active_low;

    void pulse(uint8_t r, uint8_t g, uint8_t b, int period_ms, unsigned long now) {
        float angle = (2.0f * PI * (now % period_ms)) / period_ms;
        float factor = (sin(angle) + 1.0f) / 2.0f;
        setColor(r * factor, g * factor, b * factor);
    }
};

// ==================== 全局变量 ====================
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

MotorState leftMotor;
MotorState rightMotor;

StatusLed statusLed(LED_R_PIN, LED_G_PIN, LED_B_PIN, LED_ACTIVE_LOW);
IntervalTimer controlTimer;

volatile SystemState systemState = STATE_IDLE;

// ISR 与 loop 之间的共享控制数据
volatile struct {
    float    left_torque;
    float    right_torque;
    float    torque_limit;
    uint16_t seq;
    bool     updated;       // 标记是否有新数据
} sharedControl;

volatile unsigned long lastFrameTime = 0;
volatile uint8_t errorFlags = ERR_NONE;
unsigned long safeStopEntryTime = 0;

// ==================== 函数声明 ====================
void initMotors();
void enableMotor(MotorState* motor);
void disableMotor(MotorState* motor);
void zeroMotor(MotorState* motor);
void sendMotorCommand(MotorState* motor, float torque_limit);
bool readMotorResponse(MotorState* motor);
uint8_t calcCRC8(const uint8_t* data, uint8_t len);
float floatToUint(float x, float x_min, float x_max, int bits);
float uintToFloat(unsigned int x_int, float x_min, float x_max, int bits);

void processSerialInput();
void controlISR();
void sendStatusFrame();
void sendAckFrame(uint16_t seq, uint8_t action, uint8_t result);
void transitionTo(SystemState newState);
void emergencyStop();

// ==================== CRC8 计算 ====================
uint8_t calcCRC8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
        }
    }
    return crc;
}

// ==================== 数据转换函数 ====================
float floatToUint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (x - offset) * ((float)((1 << bits) - 1)) / span;
}

float uintToFloat(unsigned int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// ==================== 初始化 ====================
void setup() {
    Serial.begin(115200);  // baud rate ignored for USB, always 480Mbps
    delay(200);

    // 初始化CAN总线
    can1.begin();
    can1.setBaudRate(CAN_BAUD);
    can1.setMaxMB(16);
    can1.enableFIFO();
    can1.enableFIFOInterrupt();

    // 初始化电机状态
    initMotors();

    // 初始化共享控制数据
    sharedControl.left_torque = 0;
    sharedControl.right_torque = 0;
    sharedControl.torque_limit = MotorParams::TORQUE_LIMIT_DEFAULT;
    sharedControl.seq = 0;
    sharedControl.updated = false;

    systemState = STATE_IDLE;

    // 启动 500Hz 控制定时器
    controlTimer.begin(controlISR, CONTROL_PERIOD_US);
}

void initMotors() {
    leftMotor.id = LEFT_MOTOR_ID;
    leftMotor.enabled = false;
    leftMotor.position = 0;
    leftMotor.velocity = 0;
    leftMotor.current = 0;
    leftMotor.torque_cmd = 0;
    leftMotor.timeout_count = 0;
    leftMotor.connected = false;

    rightMotor.id = RIGHT_MOTOR_ID;
    rightMotor.enabled = false;
    rightMotor.position = 0;
    rightMotor.velocity = 0;
    rightMotor.current = 0;
    rightMotor.torque_cmd = 0;
    rightMotor.timeout_count = 0;
    rightMotor.connected = false;
}

// ==================== 主循环 ====================
void loop() {
    // 1. 处理串口输入
    processSerialInput();

    // 2. 超时检测（仅在 ARMED 或 RUNNING 状态）
    if (systemState == STATE_ARMED || systemState == STATE_RUNNING) {
        if (lastFrameTime > 0 && (millis() - lastFrameTime > HEARTBEAT_TIMEOUT_MS)) {
            errorFlags |= ERR_HEARTBEAT;
            transitionTo(STATE_SAFE_STOP);
        }
    }

    // 3. SAFE_STOP 持续一段时间后回到 IDLE
    if (systemState == STATE_SAFE_STOP) {
        if (millis() - safeStopEntryTime >= SAFE_STOP_GRACE_MS) {
            transitionTo(STATE_IDLE);
        }
    }

    // 4. 更新 LED（每 20ms）
    static unsigned long lastLedUpdate = 0;
    if (millis() - lastLedUpdate >= 20) {
        lastLedUpdate = millis();
        statusLed.update(systemState);
    }
}

// ==================== 状态转换 ====================
void transitionTo(SystemState newState) {
    SystemState oldState = systemState;
    if (oldState == newState) return;

    switch (newState) {
        case STATE_IDLE:
            // 确保电机已禁用
            if (leftMotor.enabled) disableMotor(&leftMotor);
            if (rightMotor.enabled) disableMotor(&rightMotor);
            errorFlags = ERR_NONE;
            lastFrameTime = 0;
            break;

        case STATE_ARMED:
            // enable 电机但不发送力矩
            leftMotor.torque_cmd = 0;
            rightMotor.torque_cmd = 0;
            lastFrameTime = millis();
            break;

        case STATE_RUNNING:
            // 从 ARMED 进入，开始执行控制
            break;

        case STATE_SAFE_STOP:
            emergencyStop();
            safeStopEntryTime = millis();
            break;
    }

    systemState = newState;
}

void emergencyStop() {
    // 力矩归零
    leftMotor.torque_cmd = 0;
    rightMotor.torque_cmd = 0;

    // 发送零力矩 CAN 指令
    if (leftMotor.enabled) {
        sendMotorCommand(&leftMotor, 0);
    }
    if (rightMotor.enabled) {
        sendMotorCommand(&rightMotor, 0);
    }

    // 禁用电机
    if (leftMotor.enabled) disableMotor(&leftMotor);
    if (rightMotor.enabled) disableMotor(&rightMotor);
}

// ==================== 串口数据处理 ====================
void processSerialInput() {
    while (Serial.available()) {
        uint8_t peekByte = Serial.peek();

        if (peekByte == HEAD_CONTROL && Serial.available() >= (int)sizeof(ControlFrame)) {
            // 读取 ControlFrame
            ControlFrame frame;
            Serial.readBytes((uint8_t*)&frame, sizeof(ControlFrame));

            // 验证尾部
            if (frame.tail != FRAME_TAIL) {
                errorFlags |= ERR_CRC_FAIL;
                continue;
            }

            // 验证 CRC（从 seq 到 torque_limit，即 head 和 tail/crc 之间的所有字段）
            uint8_t* crcStart = (uint8_t*)&frame.seq;
            uint8_t crcLen = sizeof(ControlFrame) - 3;  // 减去 head, crc8, tail
            uint8_t crcCalc = calcCRC8(crcStart, crcLen);

            if (crcCalc != frame.crc8) {
                errorFlags |= ERR_CRC_FAIL;
                continue;
            }

            // 有效帧，更新心跳时间
            lastFrameTime = millis();

            // 更新共享控制数据（临界区保护）
            noInterrupts();
            sharedControl.seq = frame.seq;
            sharedControl.left_torque = frame.left_torque;
            sharedControl.right_torque = frame.right_torque;
            sharedControl.torque_limit = frame.torque_limit;
            sharedControl.updated = true;
            interrupts();

            // 状态转换: ARMED → RUNNING
            if (systemState == STATE_ARMED) {
                transitionTo(STATE_RUNNING);
            }

        } else if (peekByte == HEAD_ACTION && Serial.available() >= (int)sizeof(ActionFrame)) {
            // 读取 ActionFrame
            ActionFrame frame;
            Serial.readBytes((uint8_t*)&frame, sizeof(ActionFrame));

            // 验证尾部
            if (frame.tail != FRAME_TAIL) {
                errorFlags |= ERR_CRC_FAIL;
                continue;
            }

            // 验证 CRC
            uint8_t* crcStart = (uint8_t*)&frame.seq;
            uint8_t crcLen = sizeof(ActionFrame) - 3;  // 减去 head, crc8, tail
            uint8_t crcCalc = calcCRC8(crcStart, crcLen);

            if (crcCalc != frame.crc8) {
                errorFlags |= ERR_CRC_FAIL;
                continue;
            }

            // 处理动作
            processAction(frame);

        } else if (peekByte != HEAD_CONTROL && peekByte != HEAD_ACTION) {
            // 跳过未知字节，重新同步
            Serial.read();
        } else {
            // 帧头正确但数据不够，等待更多数据
            break;
        }
    }
}

void processAction(const ActionFrame& frame) {
    uint8_t result = RESULT_OK;

    switch (frame.action) {
        case ACTION_ENABLE:
            if (systemState == STATE_IDLE) {
                if (frame.target == TARGET_LEFT || frame.target == TARGET_BOTH) {
                    enableMotor(&leftMotor);
                }
                if (frame.target == TARGET_RIGHT || frame.target == TARGET_BOTH) {
                    enableMotor(&rightMotor);
                }
                transitionTo(STATE_ARMED);
            } else {
                result = RESULT_INVALID_STATE;
            }
            break;

        case ACTION_DISABLE:
            if (systemState == STATE_ARMED || systemState == STATE_RUNNING) {
                transitionTo(STATE_SAFE_STOP);
            } else if (systemState == STATE_SAFE_STOP) {
                // 已在安全停止中，确认 OK
            } else {
                result = RESULT_INVALID_STATE;
            }
            break;

        case ACTION_ZERO:
            if (systemState == STATE_IDLE || systemState == STATE_ARMED) {
                if (frame.target == TARGET_LEFT || frame.target == TARGET_BOTH) {
                    zeroMotor(&leftMotor);
                }
                if (frame.target == TARGET_RIGHT || frame.target == TARGET_BOTH) {
                    zeroMotor(&rightMotor);
                }
            } else {
                result = RESULT_INVALID_STATE;
            }
            break;

        case ACTION_STOP:
            // 紧急停止，任何状态都可以执行
            if (systemState == STATE_ARMED || systemState == STATE_RUNNING) {
                transitionTo(STATE_SAFE_STOP);
            }
            break;

        case ACTION_PING:
            // 连接探测，仅回复 ACK
            break;

        default:
            result = RESULT_ERROR;
            break;
    }

    sendAckFrame(frame.seq, frame.action, result);
}

// ==================== 500Hz 控制中断 ====================
void controlISR() {
    if (systemState != STATE_RUNNING) {
        // 即使不在 RUNNING 状态，也发送 StatusFrame 让 PC 知道当前状态
        sendStatusFrame();
        return;
    }

    // 读取共享控制数据
    float left_torque = sharedControl.left_torque;
    float right_torque = sharedControl.right_torque;
    float torque_limit = sharedControl.torque_limit;

    // 更新电机力矩
    leftMotor.torque_cmd = constrain(left_torque, -torque_limit, torque_limit);
    rightMotor.torque_cmd = constrain(right_torque, -torque_limit, torque_limit);

    // 发送 CAN 指令并读取反馈
    if (leftMotor.enabled) {
        sendMotorCommand(&leftMotor, torque_limit);
        leftMotor.connected = readMotorResponse(&leftMotor);
        if (!leftMotor.connected) {
            leftMotor.timeout_count++;
            if (leftMotor.timeout_count > 50) errorFlags |= ERR_LEFT_TIMEOUT;
        }
    }
    if (rightMotor.enabled) {
        sendMotorCommand(&rightMotor, torque_limit);
        rightMotor.connected = readMotorResponse(&rightMotor);
        if (!rightMotor.connected) {
            rightMotor.timeout_count++;
            if (rightMotor.timeout_count > 50) errorFlags |= ERR_RIGHT_TIMEOUT;
        }
    }

    // 发送 StatusFrame
    sendStatusFrame();
}

// ==================== 发送 StatusFrame ====================
void sendStatusFrame() {
    StatusFrame frame;
    frame.head = HEAD_STATUS;
    frame.seq = sharedControl.seq;
    frame.state = (uint8_t)systemState;
    frame.error_code = errorFlags;
    frame.left_pos = leftMotor.position;
    frame.left_vel = leftMotor.velocity;
    frame.left_cur = leftMotor.current;
    frame.right_pos = rightMotor.position;
    frame.right_vel = rightMotor.velocity;
    frame.right_cur = rightMotor.current;
    frame.left_torque_cmd = leftMotor.torque_cmd;
    frame.right_torque_cmd = rightMotor.torque_cmd;
    frame.teensy_time_us = micros();
    frame.motor_status = (leftMotor.enabled  ? 0x01 : 0) |
                         (rightMotor.enabled ? 0x02 : 0) |
                         (leftMotor.connected  ? 0x04 : 0) |
                         (rightMotor.connected ? 0x08 : 0);

    // 计算 CRC（seq 到 motor_status）
    uint8_t* crcStart = (uint8_t*)&frame.seq;
    uint8_t crcLen = sizeof(StatusFrame) - 3;  // 减去 head, crc8, tail
    frame.crc8 = calcCRC8(crcStart, crcLen);
    frame.tail = FRAME_TAIL;

    Serial.write((uint8_t*)&frame, sizeof(StatusFrame));
}

// ==================== 发送 AckFrame ====================
void sendAckFrame(uint16_t seq, uint8_t action, uint8_t result) {
    AckFrame frame;
    frame.head = HEAD_ACK;
    frame.seq = seq;
    frame.action = action;
    frame.result = result;
    frame.state = (uint8_t)systemState;

    uint8_t* crcStart = (uint8_t*)&frame.seq;
    uint8_t crcLen = sizeof(AckFrame) - 3;
    frame.crc8 = calcCRC8(crcStart, crcLen);
    frame.tail = FRAME_TAIL;

    Serial.write((uint8_t*)&frame, sizeof(AckFrame));
}

// ==================== CAN 电机通信函数 ====================
void enableMotor(MotorState* motor) {
    CAN_message_t msg;
    msg.id = motor->id;
    msg.len = 8;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFC;

    can1.write(msg);
    delayMicroseconds(500);
    can1.write(msg);

    motor->enabled = true;
    motor->timeout_count = 0;
}

void disableMotor(MotorState* motor) {
    motor->torque_cmd = 0;

    CAN_message_t msg;
    msg.id = motor->id;
    msg.len = 8;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFD;

    can1.write(msg);

    motor->enabled = false;
    motor->connected = false;
}

void zeroMotor(MotorState* motor) {
    CAN_message_t msg;
    msg.id = motor->id;
    msg.len = 8;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFE;

    can1.write(msg);
    motor->position = 0;
}

void sendMotorCommand(MotorState* motor, float torque_limit) {
    if (!motor->enabled) return;

    float limited_torque = constrain(motor->torque_cmd, -torque_limit, torque_limit);
    float i_ff = constrain(limited_torque / MotorParams::KT, -MotorParams::I_MAX, MotorParams::I_MAX);

    // 纯力矩模式: p_des=0, v_des=0, kp=0, kd=0
    uint32_t p_int = floatToUint(0, -MotorParams::P_MAX, MotorParams::P_MAX, 16);
    uint32_t v_int = floatToUint(0, -MotorParams::V_MAX, MotorParams::V_MAX, 12);
    uint32_t kp_int = 0;
    uint32_t kd_int = 0;
    uint32_t i_int = floatToUint(i_ff, -MotorParams::I_MAX, MotorParams::I_MAX, 12);

    CAN_message_t msg;
    msg.id = motor->id;
    msg.len = 8;
    msg.buf[0] = p_int >> 8;
    msg.buf[1] = p_int & 0xFF;
    msg.buf[2] = v_int >> 4;
    msg.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    msg.buf[4] = kp_int & 0xFF;
    msg.buf[5] = kd_int >> 4;
    msg.buf[6] = ((kd_int & 0xF) << 4) | (i_int >> 8);
    msg.buf[7] = i_int & 0xFF;

    can1.write(msg);
}

bool readMotorResponse(MotorState* motor) {
    CAN_message_t msg;
    unsigned long startTime = micros();
    const unsigned long timeout = 500;  // 500us 超时

    while (micros() - startTime < timeout) {
        if (can1.read(msg)) {
            if (msg.buf[0] == motor->id) {
                uint32_t p_int = (msg.buf[1] << 8) | msg.buf[2];
                uint32_t v_int = (msg.buf[3] << 4) | (msg.buf[4] >> 4);
                uint32_t i_int = ((msg.buf[4] & 0xF) << 8) | msg.buf[5];

                motor->position = uintToFloat(p_int, -MotorParams::P_MAX, MotorParams::P_MAX, 16);
                motor->velocity = uintToFloat(v_int, -MotorParams::V_MAX, MotorParams::V_MAX, 12);
                motor->current = uintToFloat(i_int, -MotorParams::I_MAX, MotorParams::I_MAX, 12);
                motor->timeout_count = 0;

                return true;
            }
        }
    }

    motor->timeout_count++;
    return false;
}
