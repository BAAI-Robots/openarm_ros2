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
测试末端位姿控制话题

该脚本会发送一系列预定义的位姿到末端位姿控制话题，
用于测试系统是否正常工作。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import math


class PoseTestPublisher(Node):
    """测试位姿发布节点"""

    def __init__(self):
        super().__init__('pose_test_publisher')
        
        self.declare_parameter('arm', 'left')
        self.arm = self.get_parameter('arm').get_parameter_value().string_value
        
        self.publisher = self.create_publisher(
            Float64MultiArray,
            f'/{self.arm}_arm/end_effector_command',
            10
        )
        
        self.get_logger().info(f'测试 {self.arm} 臂末端位姿控制')
        
    def publish_pose(self, x, y, z, qx, qy, qz, qw, gripper):
        """发布位姿"""
        msg = Float64MultiArray()
        msg.data = [x, y, z, qx, qy, qz, qw, gripper]
        self.publisher.publish(msg)
        self.get_logger().info(
            f'发送位姿: pos=({x:.3f}, {y:.3f}, {z:.3f}), '
            f'gripper={gripper:.2f}'
        )
        
    def run_test_sequence(self):
        """运行测试序列"""
        self.get_logger().info('\n开始测试序列...\n')
        time.sleep(1)
        
        # 初始位姿
        self.get_logger().info('测试 1/5: 初始位姿')
        y_offset = 0.3 if self.arm == 'left' else -0.3
        self.publish_pose(0.3, y_offset, 0.3, 0.0, 0.0, 0.0, 1.0, 0.5)
        time.sleep(5)
        
        # 向前移动
        self.get_logger().info('测试 2/5: 向前移动 10cm')
        self.publish_pose(0.4, y_offset, 0.3, 0.0, 0.0, 0.0, 1.0, 0.5)
        time.sleep(5)
        
        # 向上移动
        self.get_logger().info('测试 3/5: 向上移动 10cm')
        self.publish_pose(0.4, y_offset, 0.4, 0.0, 0.0, 0.0, 1.0, 0.5)
        time.sleep(5)
        
        # 闭合夹爪
        self.get_logger().info('测试 4/5: 闭合夹爪')
        self.publish_pose(0.4, y_offset, 0.4, 0.0, 0.0, 0.0, 1.0, 0.0)
        time.sleep(3)
        
        # 打开夹爪并返回初始位置
        self.get_logger().info('测试 5/5: 打开夹爪并返回初始位置')
        self.publish_pose(0.3, y_offset, 0.3, 0.0, 0.0, 0.0, 1.0, 1.0)
        time.sleep(5)
        
        self.get_logger().info('\n测试序列完成！\n')


def main(args=None):
    rclpy.init(args=args)
    
    node = PoseTestPublisher()
    
    try:
        node.run_test_sequence()
    except KeyboardInterrupt:
        node.get_logger().info('测试被中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
