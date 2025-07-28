#!/usr/bin/env python3
"""
Advanced debug node to collect detailed static obstacle information
Can parse complex models with multiple links and calculate real wall positions/sizes
"""

import rospy
import json
import numpy as np
import math
from datetime import datetime
from gazebo_msgs.msg import ModelStates, LinkStates
from gazebo_msgs.srv import GetModelState, GetModelProperties, GetLinkProperties
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_from_euler

class AdvancedStaticObstacleDebugger:
    def __init__(self):
        rospy.init_node('advanced_static_obstacle_debugger', anonymous=True)
        
        # Output file
        self.output_file = "/tmp/static_obstacles_debug.txt"
        self.model_states_file = "/tmp/model_states_debug.txt"
        self.link_states_file = "/tmp/link_states_debug.txt"
        self.map_data_file = "/tmp/map_data_debug.txt"
        
        # Data storage
        self.model_states_data = []
        self.link_states_data = []
        self.map_data = []
        self.model_properties = {}  # Cache for model properties
        self.link_properties = {}   # Cache for link properties
        
        # Gazebo services
        self.get_model_properties_srv = None
        self.get_model_state_srv = None
        self.get_link_properties_srv = None
        
        # Known corridor wall information (parsed from world file)
        self.corridor_walls = {
            'Wall_0': {
                'size': [41.15, 0.15, 2.5],
                'link_pose': [0, 2.5, 0, 0, 0, 0],
                'collision_pose': [0, 0, 1.25, 0, 0, 0]
            },
            'Wall_2': {
                'size': [41.15, 0.15, 2.5],
                'link_pose': [0, -2.5, 0, 0, 0, 0],
                'collision_pose': [0, 0, 1.25, 0, 0, 0]
            },
            'Wall_4': {
                'size': [5.15, 0.15, 2.5],
                'link_pose': [-20.5, 0, 0, 0, 0, -1.5708],
                'collision_pose': [0, 0, 1.25, 0, 0, 0]
            },
            'Wall_6': {
                'size': [5.15, 0.15, 2.5],
                'link_pose': [20.5, 0, 0, 0, 0, -1.5708],
                'collision_pose': [0, 0, 1.25, 0, 0, 0]
            }
        }
        
        # Initialize output files
        self.init_files()
        
        # Setup services
        self.setup_services()
        
        # Subscribers
        self.setup_subscribers()
        
        rospy.loginfo("Advanced Static Obstacle Debugger started")
        rospy.loginfo(f"Output files:")
        rospy.loginfo(f"  - Main debug: {self.output_file}")
        rospy.loginfo(f"  - Model states: {self.model_states_file}")
        rospy.loginfo(f"  - Link states: {self.link_states_file}")
        rospy.loginfo(f"  - Map data: {self.map_data_file}")
    
    def init_files(self):
        """Initialize output files with headers"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        with open(self.output_file, 'w') as f:
            f.write(f"=== Advanced Static Obstacle Debug Log ===\n")
            f.write(f"Started at: {timestamp}\n")
            f.write(f"=" * 50 + "\n\n")
        
        with open(self.model_states_file, 'w') as f:
            f.write(f"=== Gazebo Model States Debug ===\n")
            f.write(f"Started at: {timestamp}\n")
            f.write(f"=" * 50 + "\n\n")
        
        with open(self.link_states_file, 'w') as f:
            f.write(f"=== Gazebo Link States Debug ===\n")
            f.write(f"Started at: {timestamp}\n")
            f.write(f"=" * 50 + "\n\n")
        
        with open(self.map_data_file, 'w') as f:
            f.write(f"=== Map Data Debug ===\n")
            f.write(f"Started at: {timestamp}\n")
            f.write(f"=" * 50 + "\n\n")
    
    def setup_services(self):
        """Setup Gazebo services"""
        try:
            rospy.loginfo("Waiting for Gazebo services...")
            
            # Wait for model properties service
            rospy.wait_for_service('/gazebo/get_model_properties', timeout=5)
            self.get_model_properties_srv = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
            rospy.loginfo("✓ Model properties service connected")
            
            # Wait for model state service
            rospy.wait_for_service('/gazebo/get_model_state', timeout=5)
            self.get_model_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            rospy.loginfo("✓ Model state service connected")
            
            # Wait for link properties service
            rospy.wait_for_service('/gazebo/get_link_properties', timeout=5)
            self.get_link_properties_srv = rospy.ServiceProxy('/gazebo/get_link_properties', GetLinkProperties)
            rospy.loginfo("✓ Link properties service connected")
            
        except rospy.ROSException as e:
            rospy.logwarn(f"Could not connect to Gazebo services: {e}")
            rospy.logwarn("Will use basic parsing only")
    
    def transform_pose(self, pose1, pose2):
        """Transform pose2 by pose1"""
        # Convert poses to homogeneous transformation matrices
        x1, y1, z1, rx1, ry1, rz1 = pose1
        x2, y2, z2, rx2, ry2, rz2 = pose2
        
        # Create transformation matrix for pose1
        cos_rz1, sin_rz1 = math.cos(rz1), math.sin(rz1)
        cos_ry1, sin_ry1 = math.cos(ry1), math.sin(ry1)  
        cos_rx1, sin_rx1 = math.cos(rx1), math.sin(rx1)
        
        # Simplified for z-axis rotation (most common case)
        transformed_x = x1 + (x2 * cos_rz1 - y2 * sin_rz1)
        transformed_y = y1 + (x2 * sin_rz1 + y2 * cos_rz1)
        transformed_z = z1 + z2
        transformed_rz = rz1 + rz2
        
        return [transformed_x, transformed_y, transformed_z, rx1+rx2, ry1+ry2, transformed_rz]
    
    def calculate_wall_world_poses(self, model_name, model_pose):
        """Calculate real world poses of corridor walls"""
        if model_name != 'corridor':
            return {}
        
        # Convert model pose from geometry_msgs.Pose to list
        model_pose_list = [
            model_pose.position.x,
            model_pose.position.y, 
            model_pose.position.z,
            0, 0, 0  # Assuming no rotation for simplicity
        ]
        
        # If model has rotation, extract it
        if hasattr(model_pose, 'orientation'):
            q = model_pose.orientation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            model_pose_list[3:6] = [roll, pitch, yaw]
        
        wall_world_poses = {}
        
        for wall_name, wall_info in self.corridor_walls.items():
            # Calculate: model_pose + link_pose + collision_pose
            link_pose = wall_info['link_pose']
            collision_pose = wall_info['collision_pose']
            
            # Transform link pose by model pose
            link_world_pose = self.transform_pose(model_pose_list, link_pose)
            
            # Transform collision pose by link world pose
            collision_world_pose = self.transform_pose(link_world_pose, collision_pose)
            
            wall_world_poses[wall_name] = {
                'world_pose': collision_world_pose,
                'size': wall_info['size'],
                'link_pose': link_pose,
                'collision_pose': collision_pose
            }
        
        return wall_world_poses
    
    def setup_subscribers(self):
        """Setup all relevant subscribers"""
        # Gazebo model states
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        
        # Gazebo link states
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_callback)
        
        # Map related topics
        rospy.Subscriber('/dynamic_map/voxel_map', PointCloud2, self.voxel_map_callback)
        rospy.Subscriber('/dynamic_map/inflated_voxel_map', PointCloud2, self.inflated_map_callback)
        rospy.Subscriber('/dynamic_map/2D_occupancy_map', OccupancyGrid, self.occupancy_grid_callback)
        
        # Robot pose
        rospy.Subscriber('/CERLAB/quadcopter/pose', PoseStamped, self.robot_pose_callback)
        
        # Check for topic availability
        rospy.loginfo("Checking available topics...")
        import subprocess
        try:
            result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True)
            available_topics = result.stdout.split('\n')
            rospy.loginfo(f"Available topics: {len(available_topics)}")
            
            # Log some key topics
            key_topics = ['/gazebo/model_states', '/gazebo/link_states', '/dynamic_map/voxel_map', '/CERLAB/quadcopter/pose']
            for topic in key_topics:
                if topic in available_topics:
                    rospy.loginfo(f"  ✓ {topic}")
                else:
                    rospy.logwarn(f"  ✗ {topic} (not available)")
        except Exception as e:
            rospy.logwarn(f"Could not check topics: {e}")
        
        rospy.loginfo("All subscribers set up")
    
    def log_to_file(self, filename, message):
        """Log message to file with timestamp"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        with open(filename, 'a') as f:
            f.write(f"[{timestamp}] {message}\n")
    
    def estimate_model_size(self, model_name):
        """Estimate model size based on name patterns"""
        size_info = {'x': 'unknown', 'y': 'unknown', 'z': 'unknown', 'estimated': True}
        
        name_lower = model_name.lower()
        
        # Pattern matching for common obstacle names
        if 'box' in name_lower:
            parts = name_lower.split('_')
            try:
                numbers = []
                for part in parts:
                    try:
                        if '.' in part:
                            numbers.append(float(part))
                        else:
                            numbers.append(int(part))
                    except ValueError:
                        continue
                
                if len(numbers) >= 3:
                    size_info = {'x': numbers[0], 'y': numbers[1], 'z': numbers[2], 'estimated': True}
                elif len(numbers) == 1:
                    size_info = {'x': numbers[0], 'y': numbers[0], 'z': numbers[0], 'estimated': True}
                    
            except Exception:
                pass
        
        elif 'cylinder' in name_lower:
            size_info = {'type': 'cylinder', 'estimated': True}
        elif 'sphere' in name_lower:
            size_info = {'type': 'sphere', 'estimated': True}
        elif 'wall' in name_lower:
            size_info = {'type': 'wall', 'estimated': True}
        elif 'corridor' in name_lower:
            size_info = {'type': 'corridor', 'estimated': True}
            
        return size_info
    
    def model_states_callback(self, msg):
        """Process Gazebo model states"""
        timestamp = rospy.get_time()
        
        all_models = []
        static_models = []
        
        for i, name in enumerate(msg.name):
            pose = msg.pose[i]
            twist = msg.twist[i]
            
            # Get basic size estimate
            size_estimate = self.estimate_model_size(name)
            
            # Special handling for corridor - calculate wall positions
            wall_details = {}
            if name == 'corridor':
                wall_details = self.calculate_wall_world_poses(name, pose)
            
            model_info = {
                'name': name,
                'position': {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z
                },
                'orientation': {
                    'x': pose.orientation.x,
                    'y': pose.orientation.y,
                    'z': pose.orientation.z,
                    'w': pose.orientation.w
                },
                'linear_velocity': {
                    'x': twist.linear.x,
                    'y': twist.linear.y,
                    'z': twist.linear.z
                },
                'angular_velocity': {
                    'x': twist.angular.x,
                    'y': twist.angular.y,
                    'z': twist.angular.z
                },
                'size_estimate': size_estimate,
                'wall_details': wall_details
            }
            all_models.append(model_info)
            
            # Classify as static
            if ('ground_plane' not in name and 'person' not in name and 
                'quadcopter' not in name and 'unit_sphere' not in name):
                static_models.append(model_info)
        
        # Log to file
        log_msg = f"GAZEBO MODEL STATES:\n"
        log_msg += f"  Total models: {len(all_models)}\n"
        log_msg += f"  Static models: {len(static_models)}\n"
        
        log_msg += f"  STATIC MODELS DETAILED:\n"
        for model in static_models:
            pos = model['position']
            size = model['size_estimate']
            log_msg += f"    {model['name']}: pos=({pos['x']:.3f}, {pos['y']:.3f}, {pos['z']:.3f})"
            
            if size and size != {'x': 'unknown', 'y': 'unknown', 'z': 'unknown', 'estimated': True}:
                if 'type' in size:
                    log_msg += f", type={size['type']}"
                else:
                    log_msg += f", size=({size['x']}, {size['y']}, {size['z']})"
            
            # Add wall details for corridor
            if model['wall_details']:
                log_msg += f"\n      WALLS:\n"
                for wall_name, wall_info in model['wall_details'].items():
                    wp = wall_info['world_pose']
                    ws = wall_info['size']
                    log_msg += f"        {wall_name}: pos=({wp[0]:.3f}, {wp[1]:.3f}, {wp[2]:.3f}), size=({ws[0]}, {ws[1]}, {ws[2]})\n"
            else:
                log_msg += "\n"
        
        self.log_to_file(self.model_states_file, log_msg)
        
        # Log to main debug file less frequently
        if len(self.model_states_data) % 10 == 0:
            self.log_to_file(self.output_file, f"MODEL STATES UPDATE:\n{log_msg}")
        
        self.model_states_data.append({
            'timestamp': timestamp,
            'all_models': all_models,
            'static_models': static_models
        })
        
        # Print to console for immediate feedback
        if len(self.model_states_data) % 30 == 1:
            print(f"[{datetime.now().strftime('%H:%M:%S')}] Detected {len(static_models)} static models")
            
            # Print detailed wall info for corridor
            for model in static_models:
                if model['name'] == 'corridor' and model['wall_details']:
                    print(f"  Corridor walls:")
                    for wall_name, wall_info in model['wall_details'].items():
                        wp = wall_info['world_pose']
                        ws = wall_info['size']
                        print(f"    {wall_name}: ({wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f}) [{ws[0]}×{ws[1]}×{ws[2]}]")
    
    def link_states_callback(self, msg):
        """Process Gazebo link states"""
        timestamp = rospy.get_time()
        
        corridor_links = []
        for i, name in enumerate(msg.name):
            if 'corridor::' in name:
                pose = msg.pose[i]
                twist = msg.twist[i]
                
                link_info = {
                    'name': name,
                    'position': {
                        'x': pose.position.x,
                        'y': pose.position.y,
                        'z': pose.position.z
                    },
                    'orientation': {
                        'x': pose.orientation.x,
                        'y': pose.orientation.y,
                        'z': pose.orientation.z,
                        'w': pose.orientation.w
                    }
                }
                corridor_links.append(link_info)
        
        if corridor_links:
            log_msg = f"CORRIDOR LINKS ({len(corridor_links)}):\n"
            for link in corridor_links:
                pos = link['position']
                log_msg += f"  {link['name']}: pos=({pos['x']:.3f}, {pos['y']:.3f}, {pos['z']:.3f})\n"
            
            self.log_to_file(self.link_states_file, log_msg)
        
        self.link_states_data.append({
            'timestamp': timestamp,
            'corridor_links': corridor_links
        })
    
    def voxel_map_callback(self, msg):
        """Process voxel map data"""
        timestamp = rospy.get_time()
        
        info = {
            'timestamp': timestamp,
            'frame_id': msg.header.frame_id,
            'points_count': msg.width * msg.height,
            'point_step': msg.point_step,
            'row_step': msg.row_step,
            'is_dense': msg.is_dense
        }
        
        log_msg = f"VOXEL MAP: {info['points_count']} points in frame '{info['frame_id']}'"
        self.log_to_file(self.map_data_file, log_msg)
        
        if len(self.map_data) % 5 == 0:
            self.log_to_file(self.output_file, f"VOXEL MAP UPDATE: {log_msg}")
            print(f"[{datetime.now().strftime('%H:%M:%S')}] Voxel map: {info['points_count']} points")
        
        self.map_data.append(info)
    
    def inflated_map_callback(self, msg):
        """Process inflated voxel map"""
        timestamp = rospy.get_time()
        points_count = msg.width * msg.height
        
        log_msg = f"INFLATED MAP: {points_count} points in frame '{msg.header.frame_id}'"
        self.log_to_file(self.map_data_file, log_msg)
        print(f"[{datetime.now().strftime('%H:%M:%S')}] Inflated map: {points_count} points")
    
    def occupancy_grid_callback(self, msg):
        """Process 2D occupancy grid"""
        timestamp = rospy.get_time()
        
        info = {
            'timestamp': timestamp,
            'frame_id': msg.header.frame_id,
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
            'origin_z': msg.info.origin.position.z
        }
        
        occupied_cells = sum(1 for cell in msg.data if cell > 50)
        
        log_msg = f"2D OCCUPANCY GRID: {info['width']}x{info['height']} @ {info['resolution']:.3f}m/cell, "
        log_msg += f"origin=({info['origin_x']:.1f}, {info['origin_y']:.1f}), {occupied_cells} occupied cells"
        
        self.log_to_file(self.map_data_file, log_msg)
        print(f"[{datetime.now().strftime('%H:%M:%S')}] 2D Grid: {occupied_cells} occupied cells")
    
    def robot_pose_callback(self, msg):
        """Process robot pose"""
        timestamp = rospy.get_time()
        
        pose_info = {
            'timestamp': timestamp,
            'frame_id': msg.header.frame_id,
            'position': {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z
            }
        }
        
        if hasattr(self, 'last_robot_log'):
            if timestamp - self.last_robot_log > 2.0:
                log_msg = f"ROBOT POSE: ({pose_info['position']['x']:.2f}, {pose_info['position']['y']:.2f}, {pose_info['position']['z']:.2f})"
                self.log_to_file(self.output_file, log_msg)
                print(f"[{datetime.now().strftime('%H:%M:%S')}] Robot: ({pose_info['position']['x']:.2f}, {pose_info['position']['y']:.2f}, {pose_info['position']['z']:.2f})")
                self.last_robot_log = timestamp
        else:
            self.last_robot_log = timestamp
    
    def generate_summary(self):
        """Generate detailed summary of collected data"""
        summary_file = "/tmp/static_obstacles_summary.txt"
        
        with open(summary_file, 'w') as f:
            f.write("=== ADVANCED STATIC OBSTACLES DEBUG SUMMARY ===\n")
            f.write(f"Generated at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            if self.model_states_data:
                latest_data = self.model_states_data[-1]
                all_models = latest_data['all_models']
                static_models = latest_data['static_models']
                
                f.write(f"MODEL DETECTION SUMMARY:\n")
                f.write("-" * 50 + "\n")
                f.write(f"Total models detected: {len(all_models)}\n")
                f.write(f"Static models detected: {len(static_models)}\n\n")
                
                f.write(f"DETAILED STATIC MODELS:\n")
                f.write("-" * 50 + "\n")
                for model in static_models:
                    pos = model['position']
                    size = model['size_estimate']
                    f.write(f"\nModel: {model['name']}\n")
                    f.write(f"  Position: ({pos['x']:.3f}, {pos['y']:.3f}, {pos['z']:.3f})\n")
                    
                    if size and size != {'x': 'unknown', 'y': 'unknown', 'z': 'unknown', 'estimated': True}:
                        if 'type' in size:
                            f.write(f"  Type: {size['type']}\n")
                        else:
                            f.write(f"  Size: ({size['x']}, {size['y']}, {size['z']})\n")
                    
                    # Add detailed wall information
                    if model['wall_details']:
                        f.write(f"  WALL DETAILS:\n")
                        for wall_name, wall_info in model['wall_details'].items():
                            wp = wall_info['world_pose']
                            ws = wall_info['size']
                            f.write(f"    {wall_name}:\n")
                            f.write(f"      World Position: ({wp[0]:.3f}, {wp[1]:.3f}, {wp[2]:.3f})\n")
                            f.write(f"      Size: ({ws[0]}, {ws[1]}, {ws[2]})\n")
                            if len(wp) > 3:
                                f.write(f"      Rotation: ({wp[3]:.3f}, {wp[4]:.3f}, {wp[5]:.3f})\n")
                f.write("\n")
            
            if self.map_data:
                latest_map = self.map_data[-1]
                f.write(f"LATEST MAP DATA:\n")
                f.write("-" * 30 + "\n")
                f.write(f"  Frame: {latest_map['frame_id']}\n")
                f.write(f"  Points: {latest_map['points_count']}\n")
                f.write(f"  Point step: {latest_map['point_step']}\n")
                f.write(f"  Dense: {latest_map['is_dense']}\n\n")
            
            f.write(f"DATA COLLECTION STATISTICS:\n")
            f.write("-" * 30 + "\n")
            f.write(f"  Model states messages: {len(self.model_states_data)}\n")
            f.write(f"  Link states messages: {len(self.link_states_data)}\n")
            f.write(f"  Map data messages: {len(self.map_data)}\n")
        
        rospy.loginfo(f"Summary written to: {summary_file}")
        print(f"\nSummary written to: {summary_file}")
        print("You can view it with: cat /tmp/static_obstacles_summary.txt")

def main():
    try:
        debugger = AdvancedStaticObstacleDebugger()
        
        rospy.loginfo("Collecting data for 10 seconds...")
        print("Collecting advanced static obstacle data for 10 seconds...")
        print("Real-time logs will be saved to:")
        print(f"  - {debugger.output_file}")
        print(f"  - {debugger.model_states_file}") 
        print(f"  - {debugger.link_states_file}")
        print(f"  - {debugger.map_data_file}")
        print("\nWatching for messages...")
        
        start_time = rospy.get_time()
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            if current_time - start_time > 10.0:
                break
            rate.sleep()
        
        debugger.generate_summary()
        rospy.loginfo("Data collection completed!")
        print("\nData collection completed!")
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"Error: {e}")
        rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    main() 