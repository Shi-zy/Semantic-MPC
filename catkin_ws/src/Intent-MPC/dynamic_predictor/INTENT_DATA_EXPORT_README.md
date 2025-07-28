# 意图数据导出功能说明

## 功能概述

这个功能可以将动态预测器**当前检测到的障碍物**及其关键历史数据和计算得出的意图概率导出到CSV文件中，用于实时监控、数据分析和调试。每次导出只包含当前时刻的障碍物状态，数据精简高效。

## 数据内容

导出的数据包含以下信息：
- **时间戳**: 数据导出时的时间戳
- **障碍物ID**: 每个障碍物的唯一标识符
- **历史帧数**: 该障碍物的历史数据帧数
- **当前位置&速度&加速度**: 最新帧的位置、速度和加速度数据
- **前一帧位置&速度&加速度**: 前一帧的位置、速度和加速度数据
- **最旧帧位置&速度&加速度**: 历史中最旧帧的位置、速度和加速度数据
- **尺寸数据**: X, Y, Z 方向的尺寸
- **计算得出的意图概率**: 前进、左转、右转、停止四种意图的概率

## 使用方法

### 1. 启用数据导出

在配置文件中设置以下参数：

```yaml
# 在 predictor_param.yaml 中
enable_data_export: true  # 启用数据导出
export_file_path: "/path/to/your/intent_data.csv"  # 设置导出文件路径
export_interval: 10  # 每10次预测导出一次数据
```

### 2. 配置文件位置

- `catkin_ws/src/Intent-MPC/dynamic_predictor/cfg/predictor_param.yaml`
- `catkin_ws/src/Intent-MPC/autonomous_flight/cfg/mpc_navigation/predictor_param.yaml`

### 3. 运行系统

启动动态预测器后，如果启用了数据导出，系统会：
- 每隔指定次数的预测后自动导出数据
- 在终端显示导出状态信息
- 将数据保存到指定的CSV文件中

### 4. 手动导出（可选）

如果需要在代码中手动触发导出，可以调用：

```cpp
predictor_instance.exportIntentData("/path/to/export/file.csv");
```

## 数据格式

导出的CSV文件格式如下：

```
# Intent Data Export - Current Obstacles and Computed Intent
# Each row represents one currently detected obstacle with its key history data and computed intent
# Timestamp, Obstacle_ID, NumHistFrames, Current_Pos_X, Current_Pos_Y, Current_Pos_Z, Current_Vel_X, Current_Vel_Y, Current_Vel_Z, Current_Acc_X, Current_Acc_Y, Current_Acc_Z, Prev_Pos_X, Prev_Pos_Y, Prev_Pos_Z, Prev_Vel_X, Prev_Vel_Y, Prev_Vel_Z, Prev_Acc_X, Prev_Acc_Y, Prev_Acc_Z, Oldest_Pos_X, Oldest_Pos_Y, Oldest_Pos_Z, Oldest_Vel_X, Oldest_Vel_Y, Oldest_Vel_Z, Oldest_Acc_X, Oldest_Acc_Y, Oldest_Acc_Z, Size_X, Size_Y, Size_Z, Intent_Forward, Intent_Left, Intent_Right, Intent_Stop
1234567890.123456, 0, 5, 1.2, 2.3, 0.0, 0.5, 0.8, 0.0, 0.1, 0.2, 0.0, 1.1, 2.1, 0.0, 0.4, 0.7, 0.0, 0.08, 0.15, 0.0, 1.0, 2.0, 0.0, 0.3, 0.6, 0.0, 0.05, 0.1, 0.0, 0.6, 0.4, 1.8, 0.7, 0.1, 0.1, 0.1
1234567890.123456, 1, 4, 2.5, 3.1, 0.0, 0.3, 0.4, 0.0, 0.02, 0.03, 0.0, 2.4, 3.0, 0.0, 0.2, 0.3, 0.0, 0.01, 0.02, 0.0, 2.2, 2.8, 0.0, 0.1, 0.2, 0.0, 0.0, 0.01, 0.0, 0.5, 0.5, 1.8, 0.2, 0.6, 0.1, 0.1
...
```

## 意图判断逻辑

根据源代码分析，意图判断使用以下逻辑：
- 需要至少3帧历史数据才能进行意图判断
- 使用**连续三帧组合**来累积计算转移概率
- 导出数据包含**当前检测到的障碍物**及其关键历史帧：
  - **当前帧**：最新的位置、速度和加速度
  - **前一帧**：用于计算运动趋势的位置、速度和加速度
  - **最旧帧**：表示历史轨迹起点的位置、速度和加速度
- 意图概率是基于整个历史序列计算的累积结果
- 意图类型包括：前进(FORWARD)、左转(LEFT)、右转(RIGHT)、停止(STOP)
- **每行代表一个当前检测到的障碍物**及其计算得出的意图

## 注意事项

1. **性能影响**: 启用数据导出会有轻微的性能开销，建议在调试时使用
2. **存储空间**: 根据障碍物数量和历史帧数，导出文件可能会很大
3. **文件权限**: 确保指定的导出路径有写入权限
4. **数据精简**: 每行只包含一个障碍物的关键信息，避免冗余数据

## 故障排除

如果数据导出不工作，请检查：
1. `enable_data_export` 参数是否设置为 `true`
2. 导出文件路径是否有效且有写入权限
3. 系统中是否有障碍物被检测到
4. 障碍物是否有足够的历史数据（至少3帧）
5. 查看终端输出的当前障碍物统计信息

## 示例使用场景

- **算法调试**: 分析意图判断的准确性，观察当前检测结果
- **数据收集**: 为机器学习模型收集简洁的障碍物状态和意图数据
- **实时监控**: 监控当前检测到的障碍物及其运动意图
- **系统验证**: 验证预测器对当前场景的响应
- **行为分析**: 研究障碍物的当前运动状态和意图分布
- **参数调优**: 基于实际检测结果调整预测器参数 