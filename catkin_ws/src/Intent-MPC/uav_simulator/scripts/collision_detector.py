#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import String
import re

class CollisionDetector:
    def __init__(self):
        rospy.init_node('collision_detector_node', anonymous=True)
        
        # 订阅碰撞话题
        self.sub = rospy.Subscriber('/drone/contacts', ContactsState, self.collision_callback)
        
        # 可选：发布碰撞事件
        self.pub = rospy.Publisher('/drone/collision_event', String, queue_size=10)
        
        self.drone_model = "quadcopter"  # 无人机模型名称

    def extract_model_name(self, collision_str):
        # 从字符串提取模型名称 (e.g., "quadcopter::base_link::collision" -> "quadcopter")
        match = re.match(r'([^:]+)', collision_str)
        return match.group(1) if match else None

    def collision_callback(self, data):
        if not data.states:
            return  # 无碰撞
        
        for state in data.states:
            rospy.loginfo("Collision names: %s and %s", state.collision1_name, state.collision2_name)
            
            coll1_model = self.extract_model_name(state.collision1_name)
            coll2_model = self.extract_model_name(state.collision2_name)
            
            if coll1_model == self.drone_model or coll2_model == self.drone_model:
                obstacle = coll1_model if coll2_model == self.drone_model else coll2_model
                if obstacle is None:
                    obstacle = "unknown"
                msg = f"Collision detected! Drone hit {obstacle}"
                rospy.loginfo(msg)
                self.pub.publish(msg)
            else:
                continue

if __name__ == '__main__':
    try:
        detector = CollisionDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 