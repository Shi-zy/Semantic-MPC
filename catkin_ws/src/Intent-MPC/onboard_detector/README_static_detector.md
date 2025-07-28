# 静态障碍物检测器 (Static Obstacle Detector)

## 概述

静态障碍物检测器是一个从Gazebo仿真环境直接获取静态模型信息的ROS节点，旨在替代传统的体素地图聚类方法来获取静态障碍物信息。

## 核心特性

1. **直接从Gazebo获取模型信息**：通过订阅`/gazebo/model_states`话题获取所有模型状态
2. **智能模型过滤**：自动识别和分类静态障碍物，排除动态对象和地面等
3. **语义分类**：对障碍物进行语义分类（墙壁、建筑物、家具等）
4. **服务接口**：提供ROS服务供MPC规划器调用
5. **实时可视化**：在RViz中实时显示检测到的静态障碍物

## 架构设计

### 数据流
```
Gazebo模型状态 → 静态障碍物检测器 → MPC规划器
    ↓
可视化标记 → RViz
```

### 核心组件

- **staticObstacleDetector**: 主要检测器类
- **GetStaticObstacles**: 服务定义
- **StaticObstacle/StaticObstacleArray**: 消息定义

## 使用方法

### 1. 编译

```bash
cd catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 启动静态障碍物检测器

```bash
# 单独启动检测器
roslaunch onboard_detector static_detector.launch

# 或者使用完整的MPC系统
roslaunch trajectory_planner mpc_with_static_detector.launch
```

### 3. 配置参数

编辑 `cfg/static_detector_param.yaml`:

```yaml
static_detector:
  detection_range: 20.0          # 检测范围（米）
  default_safety_distance: 0.5   # 默认安全距离
  robot_frame: "map"             # 坐标系
  
  exclude_models:                # 排除的模型
    - "ground_plane"
    - "quadcopter"
    - "person"
    
  include_patterns:              # 包含的模型模式
    - "wall"
    - "obstacle"
    - "building"
```

### 4. 在MPC规划器中启用

编辑MPC配置文件，设置：
```yaml
use_static_detector: true
```

## 服务接口

### GetStaticObstacles

**请求**:
- `geometry_msgs/Point current_position`: 当前机器人位置
- `float64 range`: 检测范围
- `string semantic_filter`: 语义过滤器（可选）

**响应**:
- `map_manager/StaticObstacle[] obstacles`: 静态障碍物列表

### 示例调用

```bash
rosservice call /static_detector/get_static_obstacles \
  "current_position: {x: 0.0, y: 0.0, z: 1.0}
   range: 15.0
   semantic_filter: ''"
```

## 话题

### 发布的话题

- `/static_detector/obstacles` (`map_manager/StaticObstacleArray`): 检测到的静态障碍物
- `/static_detector/visualization` (`visualization_msgs/MarkerArray`): 可视化标记

### 订阅的话题

- `/gazebo/model_states` (`gazebo_msgs/ModelStates`): Gazebo模型状态
- `/CERLAB/quadcopter/odom` (`nav_msgs/Odometry`): 机器人里程计

## 与传统方法的对比

| 特性 | 体素地图聚类 | 静态障碍物检测器 |
|------|-------------|------------------|
| 数据源 | 点云+体素地图 | Gazebo模型状态 |
| 精度 | 受体素分辨率限制 | 精确的模型几何 |
| 计算复杂度 | 高（聚类算法） | 低（直接解析） |
| 语义信息 | 无 | 有（模型名称） |
| 实时性 | 较低 | 高 |
| 内存使用 | 高（体素存储） | 低（模型列表） |

## 障碍物分类

检测器自动将障碍物分类为：

- **WALL**: 墙壁类结构
- **BUILDING**: 建筑物
- **CORRIDOR**: 走廊结构  
- **FURNITURE**: 家具
- **OBSTACLE**: 通用障碍物

## 故障排除

### 常见问题

1. **服务调用失败**
   - 检查静态障碍物检测器是否正在运行
   - 确认Gazebo已启动并发布模型状态

2. **无障碍物检测到**
   - 检查`include_patterns`配置
   - 确认Gazebo场景中有匹配的模型

3. **MPC规划器未使用检测器**
   - 确认`use_static_detector: true`已设置
   - 检查服务客户端连接状态

### 调试工具

```bash
# 查看检测到的障碍物
rostopic echo /static_detector/obstacles

# 查看Gazebo模型状态
rostopic echo /gazebo/model_states

# 检查服务是否可用
rosservice list | grep static_detector
```

## 未来扩展

1. **详细几何提取**：利用Gazebo服务获取精确的链接几何信息
2. **动态配置**：支持运行时参数调整
3. **多机器人支持**：支持多机器人环境下的障碍物共享
4. **实际硬件支持**：扩展到真实传感器数据

## 相关文件

- **头文件**: `include/onboard_detector/staticObstacleDetector.h`
- **实现文件**: `src/staticObstacleDetector.cpp` 
- **节点文件**: `src/static_detector_node.cpp`
- **服务定义**: `srv/GetStaticObstacles.srv`
- **消息定义**: `msg/StaticObstacle.msg`, `msg/StaticObstacleArray.msg`
- **配置文件**: `cfg/static_detector_param.yaml`
- **启动文件**: `launch/static_detector.launch` 