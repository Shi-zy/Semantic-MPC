#!/bin/bash

# 静态障碍物调试启动脚本

echo "启动静态障碍物调试系统..."

# 设置环境
source /root/new/catkin_ws/devel/setup.bash

# 清理旧的调试文件
rm -f /tmp/static_obstacles_debug.txt
rm -f /tmp/model_states_debug.txt
rm -f /tmp/map_data_debug.txt
rm -f /tmp/static_obstacles_summary.txt

echo "已清理旧的调试文件"

# 启动roscore（如果未运行）
if ! pgrep -f "roscore" > /dev/null; then
    echo "启动roscore..."
    roscore &
    sleep 3
fi

# 启动Gazebo仿真环境
echo "启动Gazebo仿真环境..."
roslaunch uav_simulator uav_world.launch world_file:="$(rospack find uav_simulator)/worlds/corridor/corridor.world" gui:=true &
sleep 10

# 启动占据地图节点
echo "启动占据地图节点..."
roslaunch map_manager occupancy_map.launch config_file:="$(rospack find autonomous_flight)/cfg/mpc_navigation/mapping_param.yaml" &
sleep 5

# 启动假检测器节点
echo "启动假检测器节点..."
roslaunch onboard_detector fake_detector.launch config_file:="$(rospack find dynamic_predictor)/cfg/fake_detector_param.yaml" &
sleep 5

# 启动调试节点
echo "启动静态障碍物调试节点..."
echo "调试数据将保存到以下文件："
echo "  - /tmp/static_obstacles_debug.txt"
echo "  - /tmp/model_states_debug.txt"
echo "  - /tmp/map_data_debug.txt"
echo "  - /tmp/static_obstacles_summary.txt"
echo ""
echo "运行30秒后会自动生成总结..."

python3 /root/new/catkin_ws/debug_static_obstacles.py

echo "调试完成！查看总结文件："
echo "cat /tmp/static_obstacles_summary.txt" 