#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoRacer 键盘遥控节点
参考 Wheeltec wheeltec_robot_keyboard 实现
适配 Ackermann 转向小车（移除 OmniMode）
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

# 默认速度参数
DEFAULT_LINEAR_SPEED = 0.2   # m/s
DEFAULT_ANGULAR_SPEED = 1.0  # rad/s
LINEAR_STEP = 0.01           # m/s
ANGULAR_STEP = 0.1           # rad/s

# 平滑加减速步长
SMOOTH_LINEAR_STEP = 0.1     # m/s per iteration
SMOOTH_ANGULAR_STEP = 0.5    # rad/s per iteration

HELP_MSG = """
AutoRacer 键盘遥控
---------------------------
移动控制:
   u    i    o
   j    k    l
   m    ,    .

速度调节:
q/z : 同时增/减线速度和角速度 (10%)
w/x : 仅增/减线速度 (10%)
e/c : 仅增/减角速度 (10%)

空格/k : 急停
其他键 : 平滑停止
CTRL-C : 退出
"""

# 方向键映射: (linear_x, angular_z)
MOVE_BINDINGS = {
    'i': (1, 0),    # 前进
    ',': (-1, 0),   # 后退
    'j': (0, 1),    # 左转
    'l': (0, -1),   # 右转
    'u': (1, 1),    # 左前
    'o': (1, -1),   # 右前
    'm': (-1, 1),   # 左后
    '.': (-1, -1),  # 右后
}

# 速度调节键映射: (linear_factor, angular_factor)
SPEED_BINDINGS = {
    'q': (1.1, 1.1),  # 同时增加
    'z': (0.9, 0.9),  # 同时减少
    'w': (1.1, 1.0),  # 仅增加线速度
    'x': (0.9, 1.0),  # 仅减少线速度
    'e': (1.0, 1.1),  # 仅增加角速度
    'c': (1.0, 0.9),  # 仅减少角速度
}


def get_key(settings):
    """读取单个按键（非阻塞，超时0.1秒）"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def smooth_velocity(target, current, step):
    """平滑速度变化"""
    if target > current:
        return min(target, current + step)
    elif target < current:
        return max(target, current - step)
    return target


def main():
    # 保存终端设置
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node('autoracer_keyboard')
    qos = QoSProfile(depth=10)
    pub = node.create_publisher(Twist, 'cmd_vel', qos)

    # 速度设置
    linear_speed = DEFAULT_LINEAR_SPEED
    angular_speed = DEFAULT_ANGULAR_SPEED

    # 方向状态
    x = 0.0   # 前进/后退方向
    th = 0.0  # 转向方向

    # 目标速度
    target_linear = 0.0
    target_angular = 0.0

    # 实际控制速度（平滑后）
    control_linear = 0.0
    control_angular = 0.0

    # 无效按键计数（用于平滑停止）
    invalid_count = 0

    try:
        print(HELP_MSG)
        print(f'当前速度: 线速度 {linear_speed:.2f} m/s, 角速度 {angular_speed:.2f} rad/s')

        while True:
            key = get_key(settings)

            if key in MOVE_BINDINGS:
                x, th = MOVE_BINDINGS[key]
                invalid_count = 0

            elif key in SPEED_BINDINGS:
                lin_factor, ang_factor = SPEED_BINDINGS[key]
                linear_speed *= lin_factor
                angular_speed *= ang_factor
                print(f'当前速度: 线速度 {linear_speed:.2f} m/s, 角速度 {angular_speed:.2f} rad/s')
                invalid_count = 0

            elif key == ' ' or key == 'k':
                # 急停：立即归零
                x = 0.0
                th = 0.0
                control_linear = 0.0
                control_angular = 0.0

            elif key == '\x03':
                # Ctrl+C 退出
                break

            else:
                # 其他键：累计后平滑停止
                invalid_count += 1
                if invalid_count > 4:
                    x = 0.0
                    th = 0.0

            # 计算目标速度
            target_linear = linear_speed * x
            target_angular = angular_speed * th

            # 平滑加减速
            control_linear = smooth_velocity(
                target_linear, control_linear, SMOOTH_LINEAR_STEP)
            control_angular = smooth_velocity(
                target_angular, control_angular, SMOOTH_ANGULAR_STEP)

            # 发布 Twist 消息
            twist = Twist()
            twist.linear.x = control_linear
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular
            pub.publish(twist)

    except Exception as e:
        print(f'错误: {e}')

    finally:
        # 发送停止命令
        twist = Twist()
        pub.publish(twist)

        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
