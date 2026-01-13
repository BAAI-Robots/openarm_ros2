#!/usr/bin/env python3
# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
键盘控制末端位姿增量脚本

发送增量指令到控制节点，控制节点会根据当前实际位姿计算目标位姿

按键映射:
- W/S: 向前/向后移动 (X轴)
- A/D: 向左/向右移动 (Y轴)
- Space/X: 向上/向下移动 (Z轴)
- I/K: 绕X轴旋转 (俯仰 Pitch)
- J/L: 绕Z轴旋转 (偏航 Yaw)
- U/O: 绕Y轴旋转 (滚转 Roll)
- Q/E: 夹爪开合 (Q=闭合, E=打开)
- 1: 切换到左臂
- 2: 切换到右臂
- Esc/Ctrl+C: 退出
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import tty
import termios
import select


class KeyboardEndEffectorControl(Node):
    """键盘控制末端位姿增量节点 - 发送增量指令"""

    def __init__(self):
        super().__init__('keyboard_end_effector_control')
        
        # 当前控制的机械臂
        self.current_arm = 'left'
        
        # 发布器 - 发送增量指令
        self.left_pub = self.create_publisher(
            Float64MultiArray,
            '/left_arm/end_effector_delta_command',
            10
        )
        self.right_pub = self.create_publisher(
            Float64MultiArray,
            '/right_arm/end_effector_delta_command',
            10
        )
        
        # 移动和旋转步长
        self.translation_step = 0.01  # 1cm per keypress
        self.rotation_step = 5.0  # 5度 per keypress
        self.gripper_step = 0.1  # 10% per keypress
        
        # 终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.print_instructions()
        self.rotation_step = 5.0  # 5度 per keypress
        self.gripper_step = 0.1  # 10% per keypress
        
        # 终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.print_instructions()
        
    def print_instructions(self):
        """打印使用说明"""
        print("\n" + "="*60)
        print("键盘控制末端位姿增量")
        print("="*60)
        print("\n平移控制 (每次移动1cm):")
        print("  W/S: 前进/后退 (X轴)")
        print("  A/D: 左移/右移 (Y轴)")
        print("  Space/X: 上升/下降 (Z轴)")
        print("\n姿态控制 (每次旋转5°):")
        print("  I/K: 绕X轴旋转 (俯仰)")
        print("  J/L: 绕Z轴旋转 (偏航)")
        print("  U/O: 绕Y轴旋转 (滚转)")
        print("\n夹爪控制:")
        print("  Q: 闭合夹爪")
        print("  E: 打开夹爪")
        print("\n其他:")
        print("  1: 切换到左臂")
        print("  2: 切换到右臂")
        print("  Esc/Ctrl+C: 退出")
        print("="*60)
        print(f"\n当前控制: {self.current_arm.upper()} 臂")
        print(f"步长: 平移={self.translation_step*100:.1f}cm, 旋转={self.rotation_step:.1f}°")
        
    def get_key(self):
        """获取键盘输入（非阻塞）"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def send_delta(self, dx=0.0, dy=0.0, dz=0.0, droll=0.0, dpitch=0.0, dyaw=0.0, dgripper=0.0):
        """
        发送增量指令
        
        Args:
            dx, dy, dz: 位置增量（米）
            droll, dpitch, dyaw: 姿态增量（度）
            dgripper: 夹爪增量（0-1）
        """
        # 构造7维增量数据: [dx, dy, dz, droll, dpitch, dyaw, dgripper]
        msg = Float64MultiArray()
        msg.data = [dx, dy, dz, droll, dpitch, dyaw, dgripper]
        
        # 发布到对应的话题
        if self.current_arm == 'left':
            self.left_pub.publish(msg)
        else:
            self.right_pub.publish(msg)
        
        # 打印发送的指令
        if dx != 0 or dy != 0 or dz != 0:
            print(f"→ 平移: dx={dx*100:.1f}cm, dy={dy*100:.1f}cm, dz={dz*100:.1f}cm")
        if droll != 0 or dpitch != 0 or dyaw != 0:
            print(f"→ 旋转: roll={droll:.1f}°, pitch={dpitch:.1f}°, yaw={dyaw:.1f}°")
        if dgripper != 0:
            print(f"→ 夹爪: {dgripper:+.2f}")
            
    def switch_arm(self, arm):
        """切换控制的机械臂"""
        if arm in ['left', 'right']:
            self.current_arm = arm
            print(f"\n切换到 {arm.upper()} 臂")
    
    def run(self):
        """主循环"""
        try:
            while True:
                key = self.get_key()
                
                # 退出
                if key == '\x1b' or key == '\x03':  # Esc or Ctrl+C
                    print("\n退出...")
                    break
                
                # 平移控制
                elif key == 'w':
                    self.send_delta(dx=self.translation_step)  # +X
                elif key == 's':
                    self.send_delta(dx=-self.translation_step)  # -X
                elif key == 'a':
                    self.send_delta(dy=self.translation_step)  # +Y
                elif key == 'd':
                    self.send_delta(dy=-self.translation_step)  # -Y
                elif key == ' ':  # Space
                    self.send_delta(dz=self.translation_step)  # +Z
                elif key == 'x':  # X key for down
                    self.send_delta(dz=-self.translation_step)  # -Z
                
                # 姿态控制
                elif key == 'i':
                    self.send_delta(dpitch=self.rotation_step)  # Pitch up
                elif key == 'k':
                    self.send_delta(dpitch=-self.rotation_step)  # Pitch down
                elif key == 'j':
                    self.send_delta(dyaw=self.rotation_step)  # Yaw left
                elif key == 'l':
                    self.send_delta(dyaw=-self.rotation_step)  # Yaw right
                elif key == 'u':
                    self.send_delta(droll=self.rotation_step)  # Roll left
                elif key == 'o':
                    self.send_delta(droll=-self.rotation_step)  # Roll right
                
                # 夹爪控制
                elif key == 'q':
                    self.send_delta(dgripper=-self.gripper_step)  # Close
                elif key == 'e':
                    self.send_delta(dgripper=self.gripper_step)  # Open
                
                # 其他功能
                elif key == '1':
                    self.switch_arm('left')
                elif key == '2':
                    self.switch_arm('right')
                
        except Exception as e:
            print(f"\n发生错误: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardEndEffectorControl()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
