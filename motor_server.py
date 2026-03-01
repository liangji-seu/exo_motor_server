#!/usr/bin/env python3
"""
motor_server.py - 电机控制服务（PC 端驱动/代理层）

三层架构中间层：
- 下行：通过 USB Serial 与 Teensy 4.1 通信（packed struct 二进制协议）
- 上行：通过 TCP localhost 向用户程序提供 JSON API

功能：
- 命令即心跳：以 200Hz 持续向 Teensy 发送 ControlFrame
- 帧完整性：CRC8 校验
- 安全机制：用户断连 → 立即 disable；SIGINT/SIGTERM → 先 disable 再退出
- 状态机：DISCONNECTED → CONNECTED → ACTIVE → STOPPING

用法：
    python motor_server.py --port COM12 --tcp-port 9090
"""

import struct
import threading
import socket
import signal
import json
import time
import logging
import argparse
import sys
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any

import serial

# ==================== Windows 定时精度 ====================
if sys.platform == 'win32':
    try:
        import ctypes
        winmm = ctypes.windll.winmm
        winmm.timeBeginPeriod(1)  # 提升 sleep 精度到 1ms
    except Exception:
        pass

# ==================== 彩色日志 ====================
# ANSI 颜色码
class C:
    RESET   = '\033[0m'
    RED     = '\033[91m'
    GREEN   = '\033[92m'
    YELLOW  = '\033[93m'
    BLUE    = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN    = '\033[96m'
    WHITE   = '\033[97m'
    DIM     = '\033[2m'
    BOLD    = '\033[1m'

# 方向标识（带颜色）
TAG_TO_MCU   = f'{C.CYAN}[→ MCU]{C.RESET}'     # server → teensy
TAG_FROM_MCU = f'{C.MAGENTA}[← MCU]{C.RESET}'   # teensy → server
TAG_FROM_USR = f'{C.GREEN}[← USR]{C.RESET}'     # user → server
TAG_TO_USR   = f'{C.YELLOW}[→ USR]{C.RESET}'    # server → user
TAG_SYS      = f'{C.WHITE}[  SYS]{C.RESET}'     # 系统事件
TAG_HEART    = f'{C.BLUE}[HEART]{C.RESET}'       # 心跳状态
TAG_ERR      = f'{C.RED}[ ERR ]{C.RESET}'        # 错误

logger = logging.getLogger('motor_server')

# ==================== 全局力矩硬限制 ====================
# 根据电机型号修改此值（单位：Nm）
TORQUE_HARD_LIMIT = 1.0  # 绝对最大力矩，server 层强制钳位

# ==================== 帧协议常量 ====================
HEAD_CONTROL = 0xAA
HEAD_ACTION  = 0xBB
HEAD_STATUS  = 0xCC
HEAD_ACK     = 0xDD
FRAME_TAIL   = 0x55

# ActionFrame 动作
ACTION_ENABLE  = 0x01
ACTION_DISABLE = 0x02
ACTION_ZERO    = 0x03
ACTION_STOP    = 0x04
ACTION_PING    = 0x10

# 目标
TARGET_LEFT  = 0x00
TARGET_RIGHT = 0x01
TARGET_BOTH  = 0x02

# AckFrame 结果
RESULT_OK            = 0x00
RESULT_INVALID_STATE = 0x01
RESULT_ERROR         = 0x02

# 帧大小
CONTROL_FRAME_SIZE = 17
ACTION_FRAME_SIZE  = 7
STATUS_FRAME_SIZE  = 44
ACK_FRAME_SIZE     = 8

# Teensy 状态
TEENSY_STATE_NAMES = {0: 'IDLE', 1: 'ARMED', 2: 'RUNNING', 3: 'SAFE_STOP'}

TARGET_MAP = {'left': TARGET_LEFT, 'right': TARGET_RIGHT, 'both': TARGET_BOTH}


# ==================== CRC8 ====================
def crc8(data: bytes) -> int:
    """CRC-8-CCITT 多项式 0x07，与 Teensy 固件一致。"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc


# ==================== 帧构建/解析 ====================
def build_control_frame(seq: int, left_torque: float, right_torque: float,
                        torque_limit: float) -> bytes:
    """构建 17 字节 ControlFrame。"""
    payload = struct.pack('<H3f', seq & 0xFFFF,
                          left_torque, right_torque, torque_limit)
    crc = crc8(payload)
    return bytes([HEAD_CONTROL]) + payload + bytes([crc, FRAME_TAIL])


def build_action_frame(seq: int, action: int, target: int) -> bytes:
    """构建 7 字节 ActionFrame。"""
    payload = struct.pack('<HBB', seq & 0xFFFF, action, target)
    crc = crc8(payload)
    return bytes([HEAD_ACTION]) + payload + bytes([crc, FRAME_TAIL])


def parse_status_frame(data: bytes) -> Optional[Dict[str, Any]]:
    """解析 44 字节 StatusFrame，返回字典或 None（CRC 错误）。"""
    if len(data) != STATUS_FRAME_SIZE or data[0] != HEAD_STATUS or data[-1] != FRAME_TAIL:
        return None

    # CRC 校验：从 seq 到 motor_status（不含 head, crc8, tail）
    payload = data[1:-2]
    if crc8(payload) != data[-2]:
        return None

    # 解包
    fields = struct.unpack('<HBB8fIB', payload)
    return {
        'seq': fields[0],
        'state': fields[1],
        'error_code': fields[2],
        'left_pos': fields[3],
        'left_vel': fields[4],
        'left_cur': fields[5],
        'right_pos': fields[6],
        'right_vel': fields[7],
        'right_cur': fields[8],
        'left_torque_cmd': fields[9],
        'right_torque_cmd': fields[10],
        'teensy_time_us': fields[11],
        'motor_status': fields[12],
    }


def parse_ack_frame(data: bytes) -> Optional[Dict[str, Any]]:
    """解析 8 字节 AckFrame，返回字典或 None。"""
    if len(data) != ACK_FRAME_SIZE or data[0] != HEAD_ACK or data[-1] != FRAME_TAIL:
        return None

    payload = data[1:-2]
    if crc8(payload) != data[-2]:
        return None

    fields = struct.unpack('<HBBB', payload)
    return {
        'seq': fields[0],
        'action': fields[1],
        'result': fields[2],
        'state': fields[3],
    }


# ==================== 服务器状态 ====================
class ServerState(Enum):
    DISCONNECTED = auto()
    CONNECTED    = auto()
    ACTIVE       = auto()
    STOPPING     = auto()


@dataclass
class MotorFeedback:
    """Teensy 反馈数据。"""
    left_pos: float = 0.0
    left_vel: float = 0.0
    left_cur: float = 0.0
    right_pos: float = 0.0
    right_vel: float = 0.0
    right_cur: float = 0.0
    left_torque_cmd: float = 0.0
    right_torque_cmd: float = 0.0
    teensy_state: int = 0
    motor_status: int = 0
    error_code: int = 0
    teensy_time_us: int = 0
    last_seq: int = 0
    last_update_time: float = 0.0


@dataclass
class SharedState:
    """线程间共享状态，通过 lock 保护。"""
    lock: threading.Lock = field(default_factory=threading.Lock)

    # 服务器状态
    server_state: ServerState = ServerState.DISCONNECTED

    # 控制参数（TCP 客户端设置，TX 线程读取）
    left_torque: float = 0.0
    right_torque: float = 0.0
    torque_limit: float = 5.0

    # 待发动作队列
    pending_actions: List[Dict] = field(default_factory=list)

    # Teensy 反馈
    feedback: MotorFeedback = field(default_factory=MotorFeedback)

    # 序列号
    control_seq: int = 0
    action_seq: int = 0

    # 关闭标志
    shutdown_requested: bool = False

    # 最近的 ACK
    last_ack: Optional[Dict] = None
    ack_event: threading.Event = field(default_factory=threading.Event)

    # 帧计数器（用于心跳监控）
    tx_frame_count: int = 0
    rx_frame_count: int = 0
    rx_crc_fail_count: int = 0


# ==================== Serial TX 线程 (200Hz) ====================
def serial_tx_loop(state: SharedState, ser: serial.Serial):
    """以 200Hz 发送 ControlFrame，同时发送 pending ActionFrame。"""
    period = 1.0 / 200.0  # 5ms

    while not state.shutdown_requested:
        loop_start = time.monotonic()

        try:
            with state.lock:
                if state.server_state == ServerState.ACTIVE:
                    # TX 层也强制钳位，作为最后防线
                    lt = max(-TORQUE_HARD_LIMIT, min(TORQUE_HARD_LIMIT, state.left_torque))
                    rt = max(-TORQUE_HARD_LIMIT, min(TORQUE_HARD_LIMIT, state.right_torque))
                    tl = min(TORQUE_HARD_LIMIT, state.torque_limit)
                    frame = build_control_frame(
                        state.control_seq,
                        lt, rt, tl
                    )
                    state.control_seq += 1
                else:
                    frame = None

                # 取出待发动作
                actions = state.pending_actions.copy()
                state.pending_actions.clear()

            # 发送 ControlFrame
            if frame is not None:
                ser.write(frame)
                with state.lock:
                    state.tx_frame_count += 1

            # 发送 ActionFrame
            for action in actions:
                with state.lock:
                    state.action_seq += 1
                    seq = state.action_seq

                action_code = {
                    'ENABLE': ACTION_ENABLE,
                    'DISABLE': ACTION_DISABLE,
                    'ZERO': ACTION_ZERO,
                    'STOP': ACTION_STOP,
                    'PING': ACTION_PING,
                }.get(action.get('action', ''), ACTION_PING)

                target_code = TARGET_MAP.get(action.get('target', 'both'), TARGET_BOTH)
                af = build_action_frame(seq, action_code, target_code)
                ser.write(af)
                logger.info(f"{TAG_TO_MCU} Action: {action['action']} target={action.get('target', 'both')} seq={seq}")

        except serial.SerialException as e:
            logger.error(f"{TAG_ERR} Serial TX error: {e}")
            with state.lock:
                state.server_state = ServerState.DISCONNECTED
            break
        except Exception as e:
            logger.error(f"{TAG_ERR} TX thread error: {e}")

        # 精确 sleep
        elapsed = time.monotonic() - loop_start
        sleep_time = period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    logger.info(f"{TAG_SYS} Serial TX thread exiting")


# ==================== Serial RX 线程 ====================
def serial_rx_loop(state: SharedState, ser: serial.Serial):
    """持续读取 StatusFrame 和 AckFrame。"""
    buffer = bytearray()
    no_data_count = 0

    while not state.shutdown_requested:
        try:
            if ser.in_waiting:
                buffer.extend(ser.read(ser.in_waiting))
                no_data_count = 0

                # 尝试解析帧
                while len(buffer) >= ACK_FRAME_SIZE:
                    if buffer[0] == HEAD_STATUS and len(buffer) >= STATUS_FRAME_SIZE:
                        frame_data = bytes(buffer[:STATUS_FRAME_SIZE])
                        status = parse_status_frame(frame_data)
                        if status is not None:
                            with state.lock:
                                fb = state.feedback
                                fb.left_pos = status['left_pos']
                                fb.left_vel = status['left_vel']
                                fb.left_cur = status['left_cur']
                                fb.right_pos = status['right_pos']
                                fb.right_vel = status['right_vel']
                                fb.right_cur = status['right_cur']
                                fb.left_torque_cmd = status['left_torque_cmd']
                                fb.right_torque_cmd = status['right_torque_cmd']
                                fb.teensy_state = status['state']
                                fb.motor_status = status['motor_status']
                                fb.error_code = status['error_code']
                                fb.teensy_time_us = status['teensy_time_us']
                                fb.last_seq = status['seq']
                                fb.last_update_time = time.monotonic()
                                state.rx_frame_count += 1
                            buffer = buffer[STATUS_FRAME_SIZE:]
                        else:
                            with state.lock:
                                state.rx_crc_fail_count += 1
                            buffer = buffer[1:]  # CRC 失败，跳过重新同步

                    elif buffer[0] == HEAD_ACK and len(buffer) >= ACK_FRAME_SIZE:
                        frame_data = bytes(buffer[:ACK_FRAME_SIZE])
                        ack = parse_ack_frame(frame_data)
                        if ack is not None:
                            with state.lock:
                                state.last_ack = ack
                            state.ack_event.set()
                            logger.debug(f"{TAG_FROM_MCU} ACK: action={ack['action']}, result={ack['result']}, teensy_state={TEENSY_STATE_NAMES.get(ack['state'], '?')}")
                            buffer = buffer[ACK_FRAME_SIZE:]
                        else:
                            buffer = buffer[1:]

                    elif buffer[0] not in (HEAD_STATUS, HEAD_ACK):
                        buffer = buffer[1:]  # 跳过未知字节
                    else:
                        break  # 帧头正确但数据不够，等待

            else:
                no_data_count += 1
                time.sleep(0.0005)  # 0.5ms

                # 如果连续 2 秒没数据且处于 ACTIVE，可能串口断了
                if no_data_count > 4000:  # 2s / 0.5ms
                    no_data_count = 0
                    with state.lock:
                        if state.server_state in (ServerState.ACTIVE, ServerState.CONNECTED):
                            logger.warning(f"{TAG_ERR} No serial data for 2 seconds, possible disconnection")

        except serial.SerialException as e:
            logger.error(f"{TAG_ERR} Serial RX error: {e}")
            with state.lock:
                state.server_state = ServerState.DISCONNECTED
            break
        except Exception as e:
            logger.error(f"{TAG_ERR} RX thread error: {e}")

    logger.info(f"{TAG_SYS} Serial RX thread exiting")


# ==================== 心跳监控线程 ====================
def heartbeat_monitor_loop(state: SharedState):
    """每 2 秒打印一行心跳状态，显示链路是否存活。"""
    prev_tx = 0
    prev_rx = 0

    while not state.shutdown_requested:
        time.sleep(2.0)
        if state.shutdown_requested:
            break

        with state.lock:
            tx = state.tx_frame_count
            rx = state.rx_frame_count
            crc_fail = state.rx_crc_fail_count
            fb = state.feedback
            srv_state = state.server_state.name
            teensy_state = TEENSY_STATE_NAMES.get(fb.teensy_state, 'UNKNOWN')
            age = time.monotonic() - fb.last_update_time if fb.last_update_time > 0 else -1

        tx_rate = (tx - prev_tx) / 2.0
        rx_rate = (rx - prev_rx) / 2.0
        prev_tx = tx
        prev_rx = rx

        # 判断链路状态
        if age < 0:
            link_str = f'{C.YELLOW}NO DATA{C.RESET}'
        elif age < 0.5:
            link_str = f'{C.GREEN}ALIVE{C.RESET} ({age*1000:.0f}ms ago)'
        else:
            link_str = f'{C.RED}DEAD{C.RESET} ({age:.1f}s ago)'

        crc_str = f' {C.RED}CRC_FAIL={crc_fail}{C.RESET}' if crc_fail > 0 else ''

        logger.info(
            f"{TAG_HEART} Link:{link_str} | "
            f"TX:{tx_rate:.0f}Hz RX:{rx_rate:.0f}Hz | "
            f"Server:{srv_state} Teensy:{teensy_state}{crc_str}"
        )

    logger.info(f"{TAG_SYS} Heartbeat monitor thread exiting")


# ==================== TCP 命令处理 ====================
def process_request(request: Dict, state: SharedState) -> Dict:
    """处理来自 TCP 客户端的 JSON 命令。"""
    cmd = request.get('cmd', '')

    if cmd == 'connect':
        # 探测 Teensy 连接
        with state.lock:
            state.pending_actions.append({'action': 'PING', 'target': 'both'})
        # 等待 ACK
        state.ack_event.clear()
        if state.ack_event.wait(timeout=2.0):
            return {'status': 'ok', 'message': 'Teensy connected'}
        else:
            return {'status': 'error', 'message': 'Teensy not responding'}

    elif cmd == 'enable':
        target = request.get('target', 'both')
        if target not in TARGET_MAP:
            return {'status': 'error', 'message': f'Invalid target: {target}'}
        with state.lock:
            state.pending_actions.append({'action': 'ENABLE', 'target': target})
        # 等待 ACK
        state.ack_event.clear()
        if state.ack_event.wait(timeout=2.0):
            with state.lock:
                ack = state.last_ack
                if ack and ack['result'] == RESULT_OK:
                    state.server_state = ServerState.ACTIVE
                    return {'status': 'ok', 'message': 'Motors enabled'}
                else:
                    result_str = 'invalid_state' if ack and ack['result'] == RESULT_INVALID_STATE else 'error'
                    return {'status': 'error', 'message': f'Enable failed: {result_str}'}
        return {'status': 'error', 'message': 'Enable timeout'}

    elif cmd == 'disable':
        target = request.get('target', 'both')
        if target not in TARGET_MAP:
            return {'status': 'error', 'message': f'Invalid target: {target}'}
        with state.lock:
            state.left_torque = 0.0
            state.right_torque = 0.0
            state.pending_actions.append({'action': 'DISABLE', 'target': target})
            state.server_state = ServerState.STOPPING
        state.ack_event.clear()
        if state.ack_event.wait(timeout=2.0):
            with state.lock:
                state.server_state = ServerState.CONNECTED
            return {'status': 'ok', 'message': 'Motors disabled'}
        with state.lock:
            state.server_state = ServerState.CONNECTED
        return {'status': 'ok', 'message': 'Disable sent (no ACK)'}

    elif cmd == 'set_torque':
        left = float(request.get('left', 0.0))
        right = float(request.get('right', 0.0))
        # 全局力矩硬限制钳位
        clamped = False
        if abs(left) > TORQUE_HARD_LIMIT:
            logger.warning(f"{TAG_ERR} {C.RED}TORQUE CLAMP: left {left:+.3f} Nm → {TORQUE_HARD_LIMIT * (1 if left > 0 else -1):+.3f} Nm (limit={TORQUE_HARD_LIMIT}){C.RESET}")
            left = TORQUE_HARD_LIMIT if left > 0 else -TORQUE_HARD_LIMIT
            clamped = True
        if abs(right) > TORQUE_HARD_LIMIT:
            logger.warning(f"{TAG_ERR} {C.RED}TORQUE CLAMP: right {right:+.3f} Nm → {TORQUE_HARD_LIMIT * (1 if right > 0 else -1):+.3f} Nm (limit={TORQUE_HARD_LIMIT}){C.RESET}")
            right = TORQUE_HARD_LIMIT if right > 0 else -TORQUE_HARD_LIMIT
            clamped = True
        with state.lock:
            state.left_torque = left
            state.right_torque = right
        return {'status': 'ok', 'clamped': clamped}

    elif cmd == 'set_torque_limit':
        limit = abs(float(request.get('limit', 5.0)))
        if limit > TORQUE_HARD_LIMIT:
            logger.warning(f"{TAG_ERR} {C.RED}TORQUE LIMIT CLAMP: {limit} Nm → {TORQUE_HARD_LIMIT} Nm (hard limit){C.RESET}")
            limit = TORQUE_HARD_LIMIT
        with state.lock:
            state.torque_limit = limit
        return {'status': 'ok', 'message': f'Torque limit set to {limit} Nm'}

    elif cmd == 'get_status':
        with state.lock:
            fb = state.feedback
            teensy_state_name = TEENSY_STATE_NAMES.get(fb.teensy_state, 'UNKNOWN')
            latency = (time.monotonic() - fb.last_update_time) * 1000 if fb.last_update_time > 0 else -1

            data = {
                'server_state': state.server_state.name,
                'teensy_state': teensy_state_name,
                'left': {
                    'position': round(fb.left_pos, 4),
                    'velocity': round(fb.left_vel, 4),
                    'current': round(fb.left_cur, 4),
                    'torque_cmd': round(fb.left_torque_cmd, 4),
                    'enabled': bool(fb.motor_status & 0x01),
                    'connected': bool(fb.motor_status & 0x04),
                },
                'right': {
                    'position': round(fb.right_pos, 4),
                    'velocity': round(fb.right_vel, 4),
                    'current': round(fb.right_cur, 4),
                    'torque_cmd': round(fb.right_torque_cmd, 4),
                    'enabled': bool(fb.motor_status & 0x02),
                    'connected': bool(fb.motor_status & 0x08),
                },
                'torque_limit': state.torque_limit,
                'error_code': fb.error_code,
                'teensy_time_us': fb.teensy_time_us,
                'last_seq': fb.last_seq,
                'latency_ms': round(latency, 1),
            }
        return {'status': 'ok', 'data': data}

    elif cmd == 'zero':
        target = request.get('target', 'both')
        if target not in TARGET_MAP:
            return {'status': 'error', 'message': f'Invalid target: {target}'}
        with state.lock:
            state.pending_actions.append({'action': 'ZERO', 'target': target})
        state.ack_event.clear()
        state.ack_event.wait(timeout=2.0)
        return {'status': 'ok', 'message': 'Zero command sent'}

    elif cmd == 'stop':
        with state.lock:
            state.left_torque = 0.0
            state.right_torque = 0.0
            state.pending_actions.append({'action': 'STOP', 'target': 'both'})
        return {'status': 'ok', 'message': 'Emergency stop sent'}

    elif cmd == 'disconnect':
        return {'status': 'ok', 'message': 'Disconnect acknowledged'}

    else:
        return {'status': 'error', 'message': f'Unknown command: {cmd}'}


# ==================== TCP 客户端处理 ====================
def handle_client(conn: socket.socket, state: SharedState):
    """处理一个 TCP 客户端连接。仅允许单客户端。"""
    conn.settimeout(0.5)
    recv_buffer = ''

    try:
        while not state.shutdown_requested:
            try:
                data = conn.recv(4096)
                if not data:
                    logger.info(f"{TAG_FROM_USR} Client disconnected (connection closed)")
                    break

                recv_buffer += data.decode('utf-8')

                # 处理所有完整的 JSON 消息（换行分隔）
                while '\n' in recv_buffer:
                    line, recv_buffer = recv_buffer.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue

                    try:
                        request = json.loads(line)
                    except json.JSONDecodeError:
                        response = {'status': 'error', 'message': 'Invalid JSON'}
                        conn.sendall((json.dumps(response) + '\n').encode('utf-8'))
                        continue

                    logger.debug(f"{TAG_FROM_USR} Request: {request}")
                    response = process_request(request, state)
                    logger.debug(f"{TAG_TO_USR} Response: {response}")

                    conn.sendall((json.dumps(response) + '\n').encode('utf-8'))

                    # 如果客户端请求断开
                    if request.get('cmd') == 'disconnect':
                        logger.info(f"{TAG_FROM_USR} Client requested disconnect")
                        return

            except socket.timeout:
                continue
            except ConnectionResetError:
                logger.info(f"{TAG_FROM_USR} Client connection reset")
                break

    finally:
        # 客户端断连 → 立即禁用电机
        logger.warning(f"{TAG_ERR} Client disconnected, disabling motors for safety")
        with state.lock:
            state.left_torque = 0.0
            state.right_torque = 0.0
            state.pending_actions.append({'action': 'DISABLE', 'target': 'both'})
            if state.server_state == ServerState.ACTIVE:
                state.server_state = ServerState.STOPPING

        # 等待 Teensy 确认
        state.ack_event.clear()
        state.ack_event.wait(timeout=1.0)

        with state.lock:
            state.server_state = ServerState.CONNECTED

        try:
            conn.close()
        except Exception:
            pass


# ==================== 信号处理 ====================
def setup_signal_handlers(state: SharedState, ser: serial.Serial):
    """注册 SIGINT/SIGTERM 处理器，确保优雅关闭。"""

    def handler(signum, frame):
        logger.info(f"{TAG_SYS} Received signal {signum}, initiating graceful shutdown")

        # 立即发送 DISABLE 给 Teensy
        try:
            with state.lock:
                state.action_seq += 1
                seq = state.action_seq
            af = build_action_frame(seq, ACTION_DISABLE, TARGET_BOTH)
            ser.write(af)
            ser.flush()
        except Exception:
            pass

        state.shutdown_requested = True

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)


# ==================== 主函数 ====================
def main():
    parser = argparse.ArgumentParser(description='Exo Motor Server - 电机控制代理服务')
    parser.add_argument('--port', '-p', type=str, required=True,
                        help='Serial port (e.g., COM12 or /dev/ttyACM0)')
    parser.add_argument('--tcp-port', type=int, default=9090,
                        help='TCP port for user program IPC (default: 9090)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Serial baud rate (ignored for USB, default: 115200)')
    parser.add_argument('--log-level', default='INFO',
                        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                        help='Logging level (default: INFO)')
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%H:%M:%S'
    )

    state = SharedState()

    # 连接串口
    logger.info(f"{TAG_SYS} Connecting to serial port {args.port}...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.01)
    except serial.SerialException as e:
        logger.error(f"{TAG_ERR} Failed to open serial port: {e}")
        sys.exit(1)

    logger.info(f"{TAG_SYS} Serial port {args.port} opened")
    state.server_state = ServerState.CONNECTED

    # 注册信号处理
    setup_signal_handlers(state, ser)

    # 启动串口线程
    tx_thread = threading.Thread(target=serial_tx_loop, args=(state, ser),
                                 name='serial-tx', daemon=True)
    rx_thread = threading.Thread(target=serial_rx_loop, args=(state, ser),
                                 name='serial-rx', daemon=True)
    tx_thread.start()
    rx_thread.start()

    # 启动心跳监控线程
    hb_thread = threading.Thread(target=heartbeat_monitor_loop, args=(state,),
                                  name='heartbeat-monitor', daemon=True)
    hb_thread.start()

    # TCP 服务器
    tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_server.bind(('127.0.0.1', args.tcp_port))
    tcp_server.listen(1)  # 仅允许单客户端
    tcp_server.settimeout(1.0)

    logger.info(f"{TAG_SYS} Motor server listening on 127.0.0.1:{args.tcp_port}")
    logger.info(f"{TAG_SYS} Waiting for user program to connect...")

    try:
        while not state.shutdown_requested:
            try:
                conn, addr = tcp_server.accept()
                logger.info(f"{TAG_FROM_USR} User program connected from {addr}")
                handle_client(conn, state)
                logger.info(f"{TAG_SYS} User program session ended, waiting for next connection...")
            except socket.timeout:
                continue
    except Exception as e:
        logger.error(f"{TAG_ERR} Server error: {e}")
    finally:
        logger.info(f"{TAG_SYS} Shutting down motor server...")

        # 确保发送 DISABLE
        try:
            with state.lock:
                state.action_seq += 1
                seq = state.action_seq
            af = build_action_frame(seq, ACTION_DISABLE, TARGET_BOTH)
            ser.write(af)
            ser.flush()
            time.sleep(0.1)
        except Exception:
            pass

        state.shutdown_requested = True
        ser.close()
        tcp_server.close()
        logger.info(f"{TAG_SYS} Motor server stopped")


if __name__ == '__main__':
    main()
