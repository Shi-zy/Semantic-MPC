#!/bin/bash

# 简化版静态障碍物调试脚本

echo "启动静态障碍物调试系统..."

# 设置环境
source /root/new/catkin_ws/devel/setup.bash

# 清理旧的调试文件
rm -f /tmp/static_obstacles_debug.txt
rm -f /tmp/model_states_debug.txt
rm -f /tmp/map_data_debug.txt
rm -f /tmp/static_obstacles_summary.txt

echo "已清理旧的调试文件"

# 检查roscore状态
if pgrep -f "roscore" > /dev/null; then
    echo "检测到roscore正在运行"
else
    echo "启动roscore..."
    roscore &
    sleep 3
fi

# 等待ROS准备就绪
sleep 2

# 启动调试节点
echo "启动静态障碍物调试节点..."
echo "调试数据将保存到以下文件："
echo "  - /tmp/static_obstacles_debug.txt"
echo "  - /tmp/model_states_debug.txt"
echo "  - /tmp/map_data_debug.txt"
echo "  - /tmp/static_obstacles_summary.txt"
echo ""
echo "运行30秒后会自动生成总结..."
echo ""

python3 /root/new/catkin_ws/debug_static_obstacles.py

echo ""
echo "调试完成！查看总结文件："
echo "cat /tmp/static_obstacles_summary.txt" 