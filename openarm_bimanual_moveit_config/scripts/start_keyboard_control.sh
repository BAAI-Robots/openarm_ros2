#!/bin/bash
# 启动末端位姿控制系统

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  OpenArm 末端位姿键盘控制系统${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 检查demo.launch.py是否在运行
if ! pgrep -f "demo.launch.py" > /dev/null; then
    echo -e "${RED}错误: demo.launch.py 未运行!${NC}"
    echo ""
    echo "请先在另一个终端运行:"
    echo -e "${YELLOW}  ros2 launch openarm_bimanual_moveit_config demo.launch.py use_fake_hardware:=true${NC}"
    echo ""
    exit 1
fi

echo -e "${GREEN}✓ 检测到 demo.launch.py 正在运行${NC}"
echo ""

# 询问用户要控制哪个臂
echo "选择要控制的机械臂:"
echo "  1) 左臂 (left)"
echo "  2) 右臂 (right)"
echo "  3) 双臂 (启动两个控制器)"
echo ""
read -p "请选择 [1-3]: " choice

case $choice in
    1)
        ARM="left"
        echo ""
        echo -e "${GREEN}启动左臂末端位姿控制器...${NC}"
        echo ""
        
        # 启动左臂控制器
        gnome-terminal -- bash -c "source /home/robot/ros2_ws/install/setup.bash && \
            ros2 run openarm_bimanual_moveit_config end_effector_pose_controller.py left; \
            exec bash"
        
        sleep 2
        
        echo -e "${GREEN}启动键盘控制脚本...${NC}"
        python3 $(dirname "$0")/keyboard_control.py
        ;;
    2)
        ARM="right"
        echo ""
        echo -e "${GREEN}启动右臂末端位姿控制器...${NC}"
        echo ""
        
        # 启动右臂控制器
        gnome-terminal -- bash -c "source /home/robot/ros2_ws/install/setup.bash && \
            ros2 run openarm_bimanual_moveit_config end_effector_pose_controller.py right; \
            exec bash"
        
        sleep 2
        
        echo -e "${GREEN}启动键盘控制脚本...${NC}"
        python3 $(dirname "$0")/keyboard_control.py
        ;;
    3)
        echo ""
        echo -e "${GREEN}启动双臂末端位姿控制器...${NC}"
        echo ""
        
        # 启动左臂控制器
        gnome-terminal -- bash -c "source /home/robot/ros2_ws/install/setup.bash && \
            ros2 run openarm_bimanual_moveit_config end_effector_pose_controller.py left; \
            exec bash"
        
        sleep 1
        
        # 启动右臂控制器
        gnome-terminal -- bash -c "source /home/robot/ros2_ws/install/setup.bash && \
            ros2 run openarm_bimanual_moveit_config end_effector_pose_controller.py right; \
            exec bash"
        
        sleep 2
        
        echo -e "${GREEN}启动键盘控制脚本 (使用1/2键切换控制的臂)...${NC}"
        python3 $(dirname "$0")/keyboard_control.py
        ;;
    *)
        echo -e "${RED}无效选择!${NC}"
        exit 1
        ;;
esac
