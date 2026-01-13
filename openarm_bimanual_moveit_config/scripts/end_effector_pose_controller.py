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
末端位姿增量控制节点

订阅话题 /left_arm/end_effector_delta_command 和 /right_arm/end_effector_delta_command
接收7维增量数据：[dx, dy, dz, droll, dpitch, dyaw, dgripper]
从TF获取当前末端位姿，计算新目标位姿，使用MoveIt控制机械臂
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    MoveItErrorCodes,
)
from shape_msgs.msg import SolidPrimitive
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation
import numpy as np
import sys


class EndEffectorPoseController(Node):
    """末端位姿增量控制节点"""

    def __init__(self):
        super().__init__('end_effector_pose_controller')
        
        # 声明参数
        self.declare_parameter('arm', 'left')  # 可选 'left' 或 'right'
        self.arm = self.get_parameter('arm').get_parameter_value().string_value
        
        self.get_logger().info(f'初始化 {self.arm} 臂末端位姿增量控制节点')
        
        # 初始化TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 末端链接名称 (TCP = Tool Center Point)
        self.end_effector_link = f"openarm_{self.arm}_hand_tcp"
        
        # 当前夹爪状态
        self.current_gripper = 0.5
        
        # 初始化MoveGroup action客户端
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )
        
        self.get_logger().info('等待MoveGroup action服务器...')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action服务器未响应!')
            sys.exit(1)
        
        self.get_logger().info(f'MoveGroup action客户端已连接')
        
        # 订阅增量指令话题
        self.delta_sub = self.create_subscription(
            Float64MultiArray,
            f'/{self.arm}_arm/end_effector_delta_command',
            self.delta_callback,
            10
        )
        
        # 夹爪控制发布器
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            f'/{self.arm}_gripper_controller/commands',
            10
        )
        
        self.get_logger().info(f'订阅话题: /{self.arm}_arm/end_effector_delta_command')
        self.get_logger().info(f'发布夹爪指令到: /{self.arm}_gripper_controller/commands')
        self.get_logger().info('节点就绪，等待增量指令...')
        
    def get_current_end_effector_pose(self):
        """从TF获取当前末端位姿"""
        try:
            # 获取末端相对于world的变换
            transform = self.tf_buffer.lookup_transform(
                'world',
                self.end_effector_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 提取位置和姿态
            position = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            
            orientation = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            
            return position, orientation
            
        except TransformException as ex:
            self.get_logger().warn(f'无法获取末端位姿: {ex}')
            return None, None
    
    def delta_callback(self, msg):
        """
        接收增量指令并执行
        
        消息格式: [dx, dy, dz, droll, dpitch, dyaw, dgripper]
        - dx, dy, dz: 位置增量 (米)
        - droll, dpitch, dyaw: 姿态增量 (度)
        - dgripper: 夹爪增量 (0.0-1.0)
        """
        if len(msg.data) != 7:
            self.get_logger().warn(
                f'接收到的数据维度错误: {len(msg.data)}, 期望7维'
            )
            return
        
        # 解析增量数据
        dx, dy, dz, droll, dpitch, dyaw, dgripper = msg.data
        
        self.get_logger().info(
            f'收到增量指令: Δpos=({dx*100:.1f}, {dy*100:.1f}, {dz*100:.1f})cm, '
            f'Δrot=({droll:.1f}, {dpitch:.1f}, {dyaw:.1f})°, '
            f'Δgripper={dgripper:+.2f}'
        )
        
        # 获取当前末端位姿
        current_pos, current_quat = self.get_current_end_effector_pose()
        
        if current_pos is None:
            self.get_logger().error('无法获取当前位姿，跳过此指令')
            return
        
        self.get_logger().info(
            f'当前位姿: pos=({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})'
        )
        
        # 计算新的目标位置
        target_pos = [
            current_pos[0] + dx,
            current_pos[1] + dy,
            current_pos[2] + dz
        ]
        
        # 计算新的目标姿态
        current_rot = Rotation.from_quat(current_quat)
        delta_rot = Rotation.from_euler('xyz', [droll, dpitch, dyaw], degrees=True)
        target_rot = delta_rot * current_rot
        target_quat = target_rot.as_quat()  # [x, y, z, w]
        
        self.get_logger().info(
            f'目标位姿: pos=({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})'
        )
        
        # 发送MoveGroup指令
        self.send_move_goal(target_pos, target_quat, dgripper)
    
    def send_move_goal(self, target_pos, target_quat, dgripper):
        """
        发送MoveGroup goal
        
        消息格式: [x, y, z, qx, qy, qz, qw, gripper_position]
        - x, y, z: 位置 (米)
        - qx, qy, qz, qw: 四元数姿态
        - gripper_position: 夹爪位置 (0.0 = 闭合, 1.0 = 打开)
        """
        x, y, z = target_pos
        qx, qy, qz, qw = target_quat
        
        # 更新夹爪状态
        self.current_gripper = max(0.0, min(1.0, self.current_gripper + dgripper))
        
        # 更新夹爪状态
        self.current_gripper = max(0.0, min(1.0, self.current_gripper + dgripper))
        
        # 创建MoveGroup goal
        goal = MoveGroup.Goal()
        goal.request.group_name = f"{self.arm}_arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        
        # 设置目标位姿约束
        pose_constraint = Constraints()
        pose_constraint.name = "target_pose"
        
        # 位置约束
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "world"
        position_constraint.link_name = self.end_effector_link
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        
        # 创建包围盒
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]  # 1mm tolerance
        bounding_volume.primitives.append(sphere)
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        bounding_volume.primitive_poses.append(pose)
        
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        
        # 姿态约束
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "world"
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.01
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0
        
        pose_constraint.position_constraints.append(position_constraint)
        pose_constraint.orientation_constraints.append(orientation_constraint)
        goal.request.goal_constraints.append(pose_constraint)
        
        # 发送goal并等待结果
        self.get_logger().info('发送规划请求...')
        send_goal_future = self.move_group_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """处理goal响应"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal被拒绝')
            return
        
        self.get_logger().info('Goal被接受，等待规划结果...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        """处理执行结果"""
        result = future.result().result
        error_code = result.error_code.val
        
        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('轨迹执行成功!')
        else:
            self.get_logger().warn(f'轨迹执行失败，错误代码: {error_code}')
        
        # 控制夹爪
        self.control_gripper()
    
    def control_gripper(self):
        """
        控制夹爪位置
        """
        # 将0-1的范围映射到夹爪实际范围（假设0-0.04米）
        gripper_value = self.current_gripper * 0.04
        
        gripper_msg = Float64MultiArray()
        gripper_msg.data = [gripper_value]
        self.gripper_pub.publish(gripper_msg)
        self.get_logger().info(f'发送夹爪指令: {gripper_value:.4f}m ({self.current_gripper*100:.1f}%)')


def main(args=None):
    rclpy.init(args=args)
    
    # 检查命令行参数
    if len(sys.argv) > 1:
        arm = sys.argv[1]
        if arm not in ['left', 'right']:
            print(f"错误: arm参数必须是 'left' 或 'right', 但收到 '{arm}'")
            sys.exit(1)
    else:
        arm = 'left'  # 默认左臂
    
    node = EndEffectorPoseController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
