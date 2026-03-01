#!/usr/bin/env python3
"""
motor_client.py - 电机控制客户端库

三层架构用户层：通过 TCP 连接 motor_server，提供简洁的 Python API。

用法:
    # 方式 1：上下文管理器（推荐）
    with MotorClient() as mc:
        mc.enable()
        mc.set_torque(left=0.5, right=-0.3)
        status = mc.get_status()
        mc.disable()

    # 方式 2：手动管理
    mc = MotorClient()
    mc.connect()
    mc.enable()
    ...
    mc.disconnect()
"""

import socket
import json
import time
from typing import Optional, Dict, Any


class MotorError(Exception):
    """电机服务器返回的错误。"""
    pass


class MotorClient:
    """
    电机控制客户端。

    通过 TCP 连接 motor_server，发送 JSON 命令，接收 JSON 响应。
    支持上下文管理器，退出时自动 disable + disconnect。
    """

    def __init__(self, host: str = '127.0.0.1', port: int = 9090,
                 timeout: float = 5.0):
        """
        Args:
            host: motor_server 地址（默认 127.0.0.1）
            port: motor_server TCP 端口（默认 9090）
            timeout: 通信超时时间（秒）
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._recv_buffer = ''

    # ==================== 连接管理 ====================

    def connect(self) -> Dict:
        """
        连接到 motor_server 并探测 Teensy。

        Returns:
            服务器响应字典

        Raises:
            ConnectionError: 连接失败
            MotorError: Teensy 未响应
        """
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(self.timeout)
        try:
            self._sock.connect((self.host, self.port))
        except (ConnectionRefusedError, socket.timeout) as e:
            self._sock = None
            raise ConnectionError(f"Cannot connect to motor_server at {self.host}:{self.port}: {e}")

        self._recv_buffer = ''
        return self._send_request({'cmd': 'connect'})

    def disconnect(self):
        """优雅断开连接。先通知服务器再关闭 socket。"""
        if self._sock is None:
            return
        try:
            self._send_request({'cmd': 'disconnect'})
        except Exception:
            pass
        try:
            self._sock.close()
        except Exception:
            pass
        self._sock = None
        self._recv_buffer = ''

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self.disable()
        except Exception:
            pass
        self.disconnect()
        return False

    # ==================== 电机控制 ====================

    def enable(self, target: str = 'both') -> Dict:
        """
        使能电机。

        Args:
            target: 'left', 'right', 或 'both'

        Returns:
            服务器响应字典
        """
        return self._send_request({'cmd': 'enable', 'target': target})

    def disable(self, target: str = 'both') -> Dict:
        """
        禁用电机（安全停机）。

        Args:
            target: 'left', 'right', 或 'both'
        """
        return self._send_request({'cmd': 'disable', 'target': target})

    def set_torque(self, left: float = 0.0, right: float = 0.0) -> Dict:
        """
        设置力矩（Nm）。

        Args:
            left: 左电机力矩（Nm）
            right: 右电机力矩（Nm）
        """
        return self._send_request({'cmd': 'set_torque', 'left': left, 'right': right})

    def set_torque_limit(self, limit: float) -> Dict:
        """
        设置力矩限制（Nm）。

        Args:
            limit: 最大力矩（Nm），取绝对值
        """
        return self._send_request({'cmd': 'set_torque_limit', 'limit': limit})

    def zero(self, target: str = 'both') -> Dict:
        """
        编码器置零。

        Args:
            target: 'left', 'right', 或 'both'
        """
        return self._send_request({'cmd': 'zero', 'target': target})

    def stop(self) -> Dict:
        """紧急停止。立即将力矩置零。"""
        return self._send_request({'cmd': 'stop'})

    def get_status(self) -> Dict:
        """
        获取完整系统状态。

        Returns:
            包含以下结构的字典:
            {
                'status': 'ok',
                'data': {
                    'server_state': 'ACTIVE',
                    'teensy_state': 'RUNNING',
                    'left': {'position', 'velocity', 'current', 'torque_cmd', 'enabled', 'connected'},
                    'right': {...},
                    'torque_limit': 5.0,
                    'error_code': 0,
                    'teensy_time_us': 123456,
                    'last_seq': 100,
                    'latency_ms': 2.3
                }
            }
        """
        return self._send_request({'cmd': 'get_status'})

    # ==================== 内部通信 ====================

    def _send_request(self, request: Dict) -> Dict:
        """发送 JSON 请求并等待 JSON 响应。"""
        if self._sock is None:
            raise ConnectionError("Not connected to motor_server")

        message = json.dumps(request) + '\n'
        try:
            self._sock.sendall(message.encode('utf-8'))
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            self._sock = None
            raise ConnectionError(f"Lost connection to motor_server: {e}")

        # 读取响应（换行分隔）
        while '\n' not in self._recv_buffer:
            try:
                chunk = self._sock.recv(4096)
            except socket.timeout:
                raise MotorError("Response timeout from motor_server")
            except (ConnectionResetError, OSError) as e:
                self._sock = None
                raise ConnectionError(f"Lost connection to motor_server: {e}")

            if not chunk:
                self._sock = None
                raise ConnectionError("motor_server closed connection")

            self._recv_buffer += chunk.decode('utf-8')

        line, self._recv_buffer = self._recv_buffer.split('\n', 1)
        response = json.loads(line)

        if response.get('status') == 'error':
            raise MotorError(response.get('message', 'Unknown error'))

        return response


# ==================== 测试模式 ====================
if __name__ == '__main__':
    """
    测试模式：
        python motor_client.py test1   # 左右同向正弦力矩 (0~1Nm, 1Hz, 5秒)
        python motor_client.py test2   # 左右反向正弦力矩 (0~1Nm, 1Hz, 5秒)

    使用前确保：
    1. Teensy 已上电并通过 USB 连接 PC
    2. motor_server 已启动: python motor_server.py --port COM12
    """
    import math
    import sys as _sys

    # ---- 测试参数 ----
    AMPLITUDE = 1.0   # 正弦波幅值 (Nm)
    FREQUENCY = 1.0   # 正弦波频率 (Hz)
    DURATION  = 5.0   # 测试时长 (秒)
    LOOP_HZ   = 100   # 用户环频率

    # ---- 解析命令行 ----
    TESTS = {
        'test1': '左右同向正弦力矩',
        'test2': '左右反向正弦力矩',
    }

    if len(_sys.argv) < 2 or _sys.argv[1] not in TESTS:
        print("用法: python motor_client.py <test_name>")
        print()
        print("可用测试:")
        for name, desc in TESTS.items():
            print(f"  {name}  - {desc} ({AMPLITUDE}Nm, {FREQUENCY}Hz, {DURATION}s)")
        _sys.exit(0)

    test_name = _sys.argv[1]

    # ---- 力矩生成函数 ----
    def gen_torque_test1(t: float):
        """左右同向正弦。"""
        torque = AMPLITUDE * math.sin(2 * math.pi * FREQUENCY * t)
        return torque, torque

    def gen_torque_test2(t: float):
        """左右反向正弦。"""
        torque = AMPLITUDE * math.sin(2 * math.pi * FREQUENCY * t)
        return torque, -torque

    gen_func = {'test1': gen_torque_test1, 'test2': gen_torque_test2}[test_name]

    # ---- 执行测试 ----
    print("=" * 55)
    print(f"  Exo Motor Client - {test_name}: {TESTS[test_name]}")
    print(f"  Amplitude={AMPLITUDE}Nm  Freq={FREQUENCY}Hz  Duration={DURATION}s")
    print("=" * 55)

    try:
        with MotorClient() as mc:
            print("[OK] Connected to motor_server")

            status = mc.get_status()
            print(f"  Teensy state: {status['data']['teensy_state']}")
            print(f"  Server state: {status['data']['server_state']}")

            mc.set_torque_limit(AMPLITUDE)
            print(f"[OK] Torque limit = {AMPLITUDE} Nm")

            mc.enable()
            print("[OK] Motors enabled")

            time.sleep(0.1)
            status = mc.get_status()
            print(f"  Teensy state: {status['data']['teensy_state']}")

            print(f"\n[RUN] {TESTS[test_name]} ({DURATION}s)...")
            start = time.time()
            loop_count = 0

            while time.time() - start < DURATION:
                t = time.time() - start
                left_t, right_t = gen_func(t)
                mc.set_torque(left=left_t, right=right_t)

                # 每 0.5 秒打印一次状态
                if loop_count % (LOOP_HZ // 2) == 0:
                    status = mc.get_status()
                    d = status['data']
                    print(f"  t={t:.1f}s | "
                          f"cmd: L={left_t:+.3f} R={right_t:+.3f} Nm | "
                          f"cur: L={d['left']['current']:+.3f} R={d['right']['current']:+.3f} A | "
                          f"latency={d['latency_ms']:.1f}ms")

                loop_count += 1
                time.sleep(1.0 / LOOP_HZ)

            mc.stop()
            print(f"\n[OK] {test_name} completed")

            mc.disable()
            print("[OK] Motors disabled")

    except ConnectionError as e:
        print(f"[ERROR] Connection failed: {e}")
        print("  Make sure motor_server is running:")
        print("  python motor_server.py --port COM12")
    except MotorError as e:
        print(f"[ERROR] Motor error: {e}")
    except KeyboardInterrupt:
        print("\n[ABORT] User interrupted")
