# OpenArm 双臂末端位姿增量控制

基于MoveIt的双臂机械臂末端位姿增量控制系统，支持键盘控制和编程接口。

## 特性

✅ **增量控制** - 基于当前实际位姿的增量式操作  
✅ **实时TF反馈** - 从TF获取真实末端位置，无累积误差  
✅ **键盘操作** - 直观的WASD键盘控制  
✅ **双臂支持** - 可独立控制左右机械臂  
✅ **MoveIt集成** - 完整的轨迹规划和碰撞检测  

## 快速开始

### 1. 启动MoveIt Demo
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py use_fake_hardware:=true
```

等待RViz启动并显示机器人模型。

### 2. 启动末端控制节点
新开终端：
```bash
source ~/ros2_ws/install/setup.bash

# 控制左臂
ros2 run openarm_bimanual_moveit_config end_effector_pose_controller.py left

# 或控制右臂
ros2 run openarm_bimanual_moveit_config end_effector_pose_controller.py right

# 或同时启动两个节点（双臂）
ros2 run openarm_bimanual_moveit_config end_effector_pose_controller.py left &
ros2 run openarm_bimanual_moveit_config end_effector_pose_controller.py right &
```

### 3. 键盘控制
新开终端：
```bash
cd ~/ros2_ws/src/openarm_ros2/openarm_bimanual_moveit_config/scripts
python3 keyboard_control.py
```

## 键盘操作

### 平移控制 (步长: 1cm)
- `W` / `S` - 向前/向后 (X轴)
- `A` / `D` - 向左/向右 (Y轴)
- `Space` / `X` - 向上/向下 (Z轴)

### 旋转控制 (步长: 5°)
- `I` / `K` - 俯仰 (绕X轴)
- `J` / `L` - 偏航 (绕Z轴)
- `U` / `O` - 滚转 (绕Y轴)

### 夹爪控制
- `Q` - 闭合 (-10%)
- `E` - 打开 (+10%)

### 其他
- `1` - 切换到左臂
- `2` - 切换到右臂
- `Esc` - 退出

## 系统架构

```
┌─────────────┐         ┌──────────────────┐         ┌─────────────┐
│  键盘输入   │ ──────> │  增量控制节点    │ ──────> │   MoveIt    │
│keyboard.py  │         │ controller.py    │         │   规划执行  │
└─────────────┘         └──────────────────┘         └─────────────┘
                               ↑
                               │ 读取当前位姿
                               │
                        ┌──────────────┐
                        │   TF Tree    │
                        └──────────────┘
```

**工作流程**：
1. 键盘脚本发送增量指令到话题
2. 控制节点从TF获取当前末端位姿
3. 控制节点计算新的目标位姿 = 当前位姿 + 增量
4. 控制节点调用MoveIt进行规划和执行

## 话题接口

### 订阅话题
- `/left_arm/end_effector_delta_command` (std_msgs/Float64MultiArray)
- `/right_arm/end_effector_delta_command` (std_msgs/Float64MultiArray)

### 数据格式
7维Float64数组：`[dx, dy, dz, droll, dpitch, dyaw, dgripper]`

| 索引 | 参数 | 单位 | 说明 |
|------|------|------|------|
| 0 | dx | 米 | X轴位置增量 |
| 1 | dy | 米 | Y轴位置增量 |
| 2 | dz | 米 | Z轴位置增量 |
| 3 | droll | 度 | 绕X轴旋转增量 |
| 4 | dpitch | 度 | 绕Y轴旋转增量 |
| 5 | dyaw | 度 | 绕Z轴旋转增量 |
| 6 | dgripper | 0-1 | 夹爪开度增量 |

## 编程使用

### 命令行测试
```bash
# 向前移动5cm
ros2 topic pub --once /left_arm/end_effector_delta_command \
  std_msgs/msg/Float64MultiArray "data: [0.05, 0, 0, 0, 0, 0, 0]"

# 向上移动3cm
ros2 topic pub --once /left_arm/end_effector_delta_command \
  std_msgs/msg/Float64MultiArray "data: [0, 0, 0.03, 0, 0, 0, 0]"

# 绕Z轴旋转10度
ros2 topic pub --once /left_arm/end_effector_delta_command \
  std_msgs/msg/Float64MultiArray "data: [0, 0, 0, 0, 0, 10, 0]"

# 打开夹爪20%
ros2 topic pub --once /left_arm/end_effector_delta_command \
  std_msgs/msg/Float64MultiArray "data: [0, 0, 0, 0, 0, 0, 0.2]"
```

### Python示例
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/left_arm/end_effector_delta_command',
            10
        )
    
    def move_forward(self, distance):
        """向前移动指定距离（米）"""
        msg = Float64MultiArray()
        msg.data = [distance, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub.publish(msg)
    
    def rotate_yaw(self, angle):
        """绕Z轴旋转指定角度（度）"""
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, angle, 0.0]
        self.pub.publish(msg)

# 使用示例
rclpy.init()
controller = ArmController()

# 向前移动5cm
controller.move_forward(0.05)

# 旋转30度
controller.rotate_yaw(30.0)

rclpy.spin_once(controller, timeout_sec=0.1)
controller.destroy_node()
rclpy.shutdown()
```

## 配置参数

### 修改控制步长
编辑 `scripts/keyboard_control.py`：
```python
self.translation_step = 0.01  # 平移步长（米），默认1cm
self.rotation_step = 5.0      # 旋转步长（度），默认5度
self.gripper_step = 0.1       # 夹爪步长（0-1），默认10%
```

### 修改规划参数
编辑 `scripts/end_effector_pose_controller.py`：
```python
goal.request.num_planning_attempts = 10        # 规划尝试次数
goal.request.allowed_planning_time = 5.0       # 规划时间限制（秒）
goal.request.max_velocity_scaling_factor = 0.1 # 速度缩放（0-1）
goal.request.max_acceleration_scaling_factor = 0.1 # 加速度缩放（0-1）
```

## 文件说明

### 核心文件
- `launch/demo.launch.py` - MoveIt启动文件
- `scripts/end_effector_pose_controller.py` - 末端控制节点
- `scripts/keyboard_control.py` - 键盘控制脚本
- `scripts/test_pose_control.py` - 自动测试脚本

### 配置文件
- `config/openarm_bimanual.srdf` - MoveIt语义配置
- `config/kinematics.yaml` - 运动学配置
- `config/moveit_controllers.yaml` - 控制器配置

## 故障排除

### 问题: TF链接未找到
```
无法获取末端位姿: "openarm_left_hand_tcp" does not exist
```
**解决**: 确保demo.launch.py已正确启动，检查TF树：
```bash
ros2 run tf2_tools view_frames
```

### 问题: MoveGroup服务器未响应
```
MoveGroup action服务器未响应!
```
**解决**: 
1. 确保demo.launch.py已启动
2. 检查MoveGroup是否运行：`ros2 node list | grep move_group`
3. 重启demo.launch.py

### 问题: 规划失败
```
轨迹执行失败，错误代码: -1
```
**解决**:
- 减小移动步长
- 检查目标位置是否在工作空间内
- 检查是否有碰撞（在RViz中查看）
- 增加规划时间: `allowed_planning_time = 10.0`

### 问题: scipy未安装
```
ModuleNotFoundError: No module named 'scipy'
```
**解决**:
```bash
conda activate py310
pip install scipy
```

### 问题: 键盘控制无响应
**解决**:
- 确保键盘控制终端窗口是激活的
- 按 `1` 或 `2` 测试连接
- 检查话题是否发布：`ros2 topic echo /left_arm/end_effector_delta_command`

## 技术细节

### TF坐标系
- `world` - 世界坐标系（基准）
- `openarm_left_hand_tcp` - 左臂末端TCP
- `openarm_right_hand_tcp` - 右臂末端TCP

### MoveGroup配置
- 规划组: `left_arm`, `right_arm`
- 规划器: OMPL (RRTConnect)
- 末端执行器: `left_ee`, `right_ee`

### 控制器
- 关节轨迹控制器: `left_joint_trajectory_controller`, `right_joint_trajectory_controller`
- 夹爪控制器: `left_gripper_controller`, `right_gripper_controller`

## 开发团队

**Enactic, Inc.**  
© 2025 Apache License 2.0

## 版本历史

- **v2.0** (2026-01-12) - 增量控制，TF实时反馈
- **v1.0** - 初始版本

## 相关链接

- [MoveIt2 文档](https://moveit.picknik.ai/main/index.html)
- [ROS2 文档](https://docs.ros.org/en/humble/index.html)
- [OpenArm 硬件](https://github.com/yourusername/openarm)
