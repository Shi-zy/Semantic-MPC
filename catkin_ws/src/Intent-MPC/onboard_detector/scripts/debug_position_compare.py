#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import tf

class PositionComparator:
    def __init__(self):
        rospy.init_node('position_comparator', anonymous=True)
        
        # 订阅Gazebo模型状态
        self.gazebo_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        
        # 订阅静态障碍物检测器的可视化
        self.static_detector_sub = rospy.Subscriber('/static_detector/visualization', MarkerArray, self.static_detector_callback)
        
        # 订阅MPC规划器的静态障碍物可视化
        self.mpc_static_sub = rospy.Subscriber('/mpc_planner/static_obstacles', MarkerArray, self.mpc_static_callback)
        
        # 存储数据
        self.gazebo_models = {}
        self.static_detector_obstacles = {}
        self.mpc_static_obstacles = {}
        
        # 定时器
        self.timer = rospy.Timer(rospy.Duration(2.0), self.compare_positions)
        
        rospy.loginfo("位置比较器已启动")
    
    def gazebo_callback(self, msg):
        """处理Gazebo模型状态"""
        for i, name in enumerate(msg.name):
            if name not in ['ground_plane', 'quadcopter']:  # 排除地面和无人机
                pose = msg.pose[i]
                self.gazebo_models[name] = {
                    'position': [pose.position.x, pose.position.y, pose.position.z],
                    'orientation': [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                }
    
    def static_detector_callback(self, msg):
        """处理静态障碍物检测器的可视化"""
        for marker in msg.markers:
            if marker.ns == "static_obstacles":
                name = f"static_detector_{marker.id}"
                self.static_detector_obstacles[name] = {
                    'position': [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z],
                    'size': [marker.scale.x, marker.scale.y, marker.scale.z]
                }
    
    def mpc_static_callback(self, msg):
        """处理MPC规划器的静态障碍物可视化"""
        for marker in msg.markers:
            if marker.ns == "mpc_static_detector_boxes":
                name = f"mpc_static_{marker.id}"
                # 从线框标记中提取中心位置（需要计算）
                if len(marker.points) >= 8:  # 确保有足够的点来定义立方体
                    # 计算立方体的中心位置
                    x_coords = [p.x for p in marker.points]
                    y_coords = [p.y for p in marker.points]
                    z_coords = [p.z for p in marker.points]
                    
                    center_x = sum(x_coords) / len(x_coords)
                    center_y = sum(y_coords) / len(y_coords)
                    center_z = sum(z_coords) / len(z_coords)
                    
                    self.mpc_static_obstacles[name] = {
                        'position': [center_x, center_y, center_z],
                        'size': [max(x_coords) - min(x_coords), max(y_coords) - min(y_coords), max(z_coords) - min(z_coords)]
                    }
    
    def compare_positions(self, event):
        """比较不同来源的位置"""
        rospy.loginfo("=== 位置比较报告 ===")
        
        # 显示Gazebo模型
        rospy.loginfo(f"Gazebo模型数量: {len(self.gazebo_models)}")
        for name, data in self.gazebo_models.items():
            rospy.loginfo(f"  Gazebo {name}: pos({data['position'][0]:.3f}, {data['position'][1]:.3f}, {data['position'][2]:.3f})")
        
        # 显示静态检测器输出
        rospy.loginfo(f"静态检测器障碍物数量: {len(self.static_detector_obstacles)}")
        for name, data in self.static_detector_obstacles.items():
            rospy.loginfo(f"  {name}: pos({data['position'][0]:.3f}, {data['position'][1]:.3f}, {data['position'][2]:.3f}), size({data['size'][0]:.3f}, {data['size'][1]:.3f}, {data['size'][2]:.3f})")
        
        # 显示MPC静态障碍物
        rospy.loginfo(f"MPC静态障碍物数量: {len(self.mpc_static_obstacles)}")
        for name, data in self.mpc_static_obstacles.items():
            rospy.loginfo(f"  {name}: pos({data['position'][0]:.3f}, {data['position'][1]:.3f}, {data['position'][2]:.3f}), size({data['size'][0]:.3f}, {data['size'][1]:.3f}, {data['size'][2]:.3f})")
        
        # 比较位置差异
        rospy.loginfo("=== 位置差异分析 ===")
        for gazebo_name, gazebo_data in self.gazebo_models.items():
            # 查找对应的静态检测器障碍物
            for detector_name, detector_data in self.static_detector_obstacles.items():
                # 简单的距离匹配（可以改进为更复杂的匹配算法）
                gazebo_pos = np.array(gazebo_data['position'])
                detector_pos = np.array(detector_data['position'])
                distance = np.linalg.norm(gazebo_pos - detector_pos)
                
                if distance < 0.1:  # 如果距离小于10cm，认为是同一个物体
                    rospy.loginfo(f"匹配: Gazebo {gazebo_name} <-> {detector_name}")
                    rospy.loginfo(f"  位置差异: {distance:.3f} m")
                    rospy.loginfo(f"  Gazebo: ({gazebo_pos[0]:.3f}, {gazebo_pos[1]:.3f}, {gazebo_pos[2]:.3f})")
                    rospy.loginfo(f"  检测器: ({detector_pos[0]:.3f}, {detector_pos[1]:.3f}, {detector_pos[2]:.3f})")
                    break
        
        rospy.loginfo("=== 报告结束 ===\n")

if __name__ == '__main__':
    try:
        comparator = PositionComparator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 