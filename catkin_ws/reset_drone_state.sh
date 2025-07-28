#!/bin/bash

# 重置无人机状态脚本
echo "重置无人机状态..."

# 重置无人机到起始位置
rostopic pub -1 /gazebo/set_model_state gazebo_msgs/ModelState "
model_name: 'quadcopter'
pose:
  position: {x: 0.0, y: 0.0, z: 0.5}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
twist:
  linear: {x: 0.0, y: 0.0, z: 0.0}
  angular: {x: 0.0, y: 0.0, z: 0.0}
"

echo "无人机状态重置完成！" 