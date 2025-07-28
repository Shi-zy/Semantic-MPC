#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import Pose, Twist
import math
import time

# 配置参数（可根据需要调整）
DRONE_MODEL_NAME = "quadcopter"  # 从launch文件确认的无人机模型名称
PERSON_MODEL_NAMES = [
    "person3_0.5_0.5_1.8",
    "person4_0.5_0.5_1.8",
    "person6_0.5_0.5_1.8",
    # 添加更多person模型名称
]  # 多人模型名称列表
# PERSON_MODEL_NAMES = [
#     "box_0.5_0.5_1.8_3",
#     "box_0.5_0.5_1.8_4",
#     "box_0.5_0.5_1.8_6",
#     # 添加更多person模型名称
# ]  # 多人模型名称列表

DISTANCE_THRESHOLD = 3.0  # 触发距离阈值（米）
STOP_DISTANCE = 3.0  # 停止追踪并返回的距离阈值（米）
MOVEMENT_SPEED = 1.2  # 运动速度（m/s）
UPDATE_RATE = 50  # 更新频率（Hz），更高的频率以实现平滑运动
RETURN_TOLERANCE = 0.1  # 返回原点时的容差（米）

class PersonMover:
    def __init__(self):
        rospy.init_node('person_mover_node', anonymous=True)
        
        # 订阅Gazebo模型状态
        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.states_callback)
        
        # 服务代理：设置模型状态
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        self.drone_pose = None
        self.person_poses = {}  # 字典存储每个人的pose
        self.person_states = {}  # 字典存储每个人的状态
        for name in PERSON_MODEL_NAMES:
            self.person_states[name] = {
                'initial_pose': None,
                'triggered': False,
                'returning': False,
                'last_update_time': time.time()
            }

    def states_callback(self, data):
        try:
            # 查找无人机索引
            drone_index = data.name.index(DRONE_MODEL_NAME)
            self.drone_pose = data.pose[drone_index]
            
            # 查找每个人的pose
            for name in PERSON_MODEL_NAMES:
                person_index = data.name.index(name)
                self.person_poses[name] = data.pose[person_index]
        except ValueError:
            rospy.logwarn("Model not found in /gazebo/model_states")
            return

    def compute_distance(self, person_pose):
        if self.drone_pose is None or person_pose is None:
            return float('inf')
        
        dx = self.drone_pose.position.x - person_pose.position.x
        dy = self.drone_pose.position.y - person_pose.position.y
        dz = self.drone_pose.position.z - person_pose.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def move_towards_drone(self, name):
        person_pose = self.person_poses.get(name)
        state = self.person_states.get(name)
        if person_pose is None or state is None:
            return
        
        # 计算方向向量
        dx = self.drone_pose.position.x - person_pose.position.x
        dy = self.drone_pose.position.y - person_pose.position.y
        dz = self.drone_pose.position.z - person_pose.position.z
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if dist == 0:
            return  # 避免除零
        
        # 单位向量
        unit_dx = dx / dist
        unit_dy = dy / dist
        unit_dz = dz / dist
        
        # 计算时间步
        current_time = time.time()
        dt = current_time - state['last_update_time']
        state['last_update_time'] = current_time
        
        # 计算移动步长
        step = MOVEMENT_SPEED * dt
        
        # 更新位置
        new_x = person_pose.position.x + unit_dx * step
        new_y = person_pose.position.y + unit_dy * step
        new_z = person_pose.position.z + unit_dz * step
        
        # 可选：计算新朝向（yaw）
        new_yaw = math.atan2(dy, dx)  # 朝着无人机方向
        
        # 设置新状态
        srv_state = SetModelStateRequest()
        srv_state.model_state.model_name = name
        srv_state.model_state.pose.position.x = new_x
        srv_state.model_state.pose.position.y = new_y
        srv_state.model_state.pose.position.z = new_z
        srv_state.model_state.pose.orientation.x = 0
        srv_state.model_state.pose.orientation.y = 0
        srv_state.model_state.pose.orientation.z = math.sin(new_yaw / 2)
        srv_state.model_state.pose.orientation.w = math.cos(new_yaw / 2)
        srv_state.model_state.twist = Twist()  # 无额外速度
        
        try:
            self.set_state_srv(srv_state)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def move_towards_initial(self, name):
        person_pose = self.person_poses.get(name)
        state = self.person_states.get(name)
        if state['initial_pose'] is None or person_pose is None:
            return False
        
        # 计算到初始位置的方向向量
        dx = state['initial_pose'].position.x - person_pose.position.x
        dy = state['initial_pose'].position.y - person_pose.position.y
        dz = state['initial_pose'].position.z - person_pose.position.z
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if dist < RETURN_TOLERANCE:
            # 已到达原点，重置状态
            rospy.loginfo(f"{name} returned to initial position.")
            return True  # 表示返回完成
        
        if dist == 0:
            return False
        
        # 单位向量
        unit_dx = dx / dist
        unit_dy = dy / dist
        unit_dz = dz / dist
        
        # 计算时间步
        current_time = time.time()
        dt = current_time - state['last_update_time']
        state['last_update_time'] = current_time
        
        # 计算移动步长
        step = MOVEMENT_SPEED * dt
        
        # 更新位置
        new_x = person_pose.position.x + unit_dx * step
        new_y = person_pose.position.y + unit_dy * step
        new_z = person_pose.position.z + unit_dz * step
        
        # 计算新朝向（yaw）
        new_yaw = math.atan2(dy, dx)
        
        # 设置新状态
        srv_state = SetModelStateRequest()
        srv_state.model_state.model_name = name
        srv_state.model_state.pose.position.x = new_x
        srv_state.model_state.pose.position.y = new_y
        srv_state.model_state.pose.position.z = new_z
        srv_state.model_state.pose.orientation.x = state['initial_pose'].orientation.x
        srv_state.model_state.pose.orientation.y = state['initial_pose'].orientation.y
        srv_state.model_state.pose.orientation.z = state['initial_pose'].orientation.z
        srv_state.model_state.pose.orientation.w = state['initial_pose'].orientation.w
        srv_state.model_state.twist = Twist()  # 无额外速度
        
        try:
            self.set_state_srv(srv_state)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        
        return False  # 返回未完成

    def run(self):
        rate = rospy.Rate(UPDATE_RATE)
        while not rospy.is_shutdown():
            for name in PERSON_MODEL_NAMES:
                person_pose = self.person_poses.get(name)
                state = self.person_states.get(name)
                if person_pose is None or state is None:
                    continue
                
                distance = self.compute_distance(person_pose)
                if distance < DISTANCE_THRESHOLD and not state['triggered'] and not state['returning']:
                    rospy.loginfo(f"Drone is close! {name} is moving towards the drone.")
                    state['initial_pose'] = Pose()
                    state['initial_pose'].position = person_pose.position
                    state['initial_pose'].orientation = person_pose.orientation
                    state['triggered'] = True
                    state['returning'] = False
                    state['last_update_time'] = time.time()
                
                if state['triggered'] and not state['returning']:
                    if distance > STOP_DISTANCE:
                        rospy.loginfo(f"Drone is too far! {name} returning to initial position.")
                        state['returning'] = True
                    else:
                        self.move_towards_drone(name)
                
                if state['returning']:
                    if self.move_towards_initial(name):
                        state['triggered'] = False
                        state['returning'] = False
                        state['initial_pose'] = None
            
            rate.sleep()

if __name__ == '__main__':
    try:
        mover = PersonMover()
        mover.run()
    except rospy.ROSInterruptException:
        pass 