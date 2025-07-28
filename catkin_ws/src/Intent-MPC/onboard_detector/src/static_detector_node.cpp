/*
	FILE: static_detector_node.cpp
	-------------------------------
	ROS node for static obstacle detector
*/

#include <ros/ros.h>
#include <onboard_detector/staticObstacleDetector.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "static_obstacle_detector");
	ros::NodeHandle nh;
	
	ROS_INFO("Starting Static Obstacle Detector Node...");
	
	onboardDetector::staticObstacleDetector detector(nh);
	
	ROS_INFO("Static Obstacle Detector Node started successfully");
	
	ros::spin();
	
	return 0;
} 