#!/usr/bin/env python3

import rospy
import rosparam
from onboard_detector.srv import GetStaticObstacles
from geometry_msgs.msg import Vector3

def test_static_detector():
    """测试静态障碍物检测器服务"""
    print("🔍 测试静态障碍物检测器...")
    
    rospy.init_node('test_static_detector', anonymous=True)
    
    # 1. 检查MPC规划器配置
    print("\n1️⃣ 检查MPC规划器配置:")
    try:
        use_static_detector = rospy.get_param('/use_static_detector', None)
        if use_static_detector is None:
            print("❌ 未找到 /use_static_detector 参数")
        elif use_static_detector:
            print("✅ MPC规划器已配置为使用静态障碍物检测器")
        else:
            print("⚠️  MPC规划器配置为使用传统聚类方法")
    except Exception as e:
        print(f"❌ 检查配置时出错: {e}")
    
    # 2. 检查静态障碍物检测器服务
    print("\n2️⃣ 检查静态障碍物检测器服务:")
    service_name = '/static_detector/get_static_obstacles'
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
        print("✅ 静态障碍物检测器服务可用")
        
        # 3. 调用服务测试
        print("\n3️⃣ 测试静态障碍物检测:")
        get_obstacles = rospy.ServiceProxy(service_name, GetStaticObstacles)
        
        # 创建测试请求
        request = GetStaticObstacles._request_class()
        request.current_position = Vector3(0, 0, 1)  # 机器人位置
        request.range = 50.0  # 检测范围50米
        request.semantic_filter = ""  # 不过滤，获取所有障碍物
        
        response = get_obstacles(request)
        
        print(f"✅ 检测到 {len(response.positions)} 个静态障碍物:")
        for i, (name, pos, size) in enumerate(zip(response.names, response.positions, response.sizes)):
            print(f"   障碍物 {i+1}: {name}")
            print(f"     位置: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
            print(f"     尺寸: ({size.x:.2f}, {size.y:.2f}, {size.z:.2f})")
            print(f"     语义类别: {response.semantic_classes[i]}")
            print()
        
        if len(response.positions) == 0:
            print("⚠️  未检测到任何静态障碍物，这可能是问题所在！")
            print("🔧 建议检查:")
            print("   - Gazebo是否正在运行？")
            print("   - world文件是否包含静态障碍物？")
            print("   - 检测器的include_patterns配置是否正确？")
        
    except rospy.ROSException:
        print("❌ 静态障碍物检测器服务不可用")
        print("🔧 建议启动静态障碍物检测器：")
        print("   roslaunch onboard_detector static_detector.launch")
    except Exception as e:
        print(f"❌ 调用服务时出错: {e}")
    
    # 4. 检查是否有其他相关的参数
    print("\n4️⃣ 检查其他相关参数:")
    try:
        # 检查静态安全距离
        static_safety_dist = rospy.get_param('/static_safety_dist', None)
        if static_safety_dist:
            print(f"✅ 静态安全距离: {static_safety_dist}m")
        
        # 检查动态安全距离  
        dynamic_safety_dist = rospy.get_param('/dynamic_safety_dist', None)
        if dynamic_safety_dist:
            print(f"✅ 动态安全距离: {dynamic_safety_dist}m")
            
    except Exception as e:
        print(f"❌ 检查参数时出错: {e}")

def print_solution():
    """打印解决方案"""
    print("\n" + "="*60)
    print("🔧 解决方案:")
    print("="*60)
    print("1. 确保使用正确的launch文件:")
    print("   roslaunch trajectory_planner mpc_with_static_detector.launch")
    print()
    print("2. 或者手动加载静态障碍物检测器配置:")
    print("   rosparam load $(rospack find trajectory_planner)/cfg/mpc_static_detector.yaml")
    print()
    print("3. 检查静态障碍物检测器是否正在运行:")
    print("   rosservice list | grep static_detector")
    print()
    print("4. 如果仍有问题，编译并重启系统:")
    print("   cd catkin_ws && catkin_make")
    print("   然后重新启动所有节点")

if __name__ == '__main__':
    try:
        test_static_detector()
        print_solution()
    except rospy.ROSInterruptException:
        pass 