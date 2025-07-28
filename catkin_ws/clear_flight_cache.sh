#!/bin/bash

# 清理飞行缓存脚本
echo "清理飞行缓存和临时文件..."

# 清理临时文件
rm -f /tmp/static_obstacles_debug.txt
rm -f /tmp/model_states_debug.txt
rm -f /tmp/link_states_debug.txt
rm -f /tmp/map_data_debug.txt
rm -f /tmp/static_obstacles_summary.txt
rm -f /tmp/intent_data.csv

# 清理地图文件
rm -f ./static_map.pcd
rm -f /tmp/*.pcd

# 清理ROS日志（可选）
# rosclean purge -y

echo "缓存清理完成！"
echo "现在可以重新启动节点了。" 