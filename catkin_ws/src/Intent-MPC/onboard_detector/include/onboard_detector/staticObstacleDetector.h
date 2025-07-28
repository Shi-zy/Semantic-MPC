/*
	FILE: staticObstacleDetector.h
	------------------------------
	Static obstacle detector for gazebo simulation
	Replaces voxel-based map by directly reading static models from Gazebo
*/
#ifndef STATIC_OBSTACLE_DETECTOR_H
#define STATIC_OBSTACLE_DETECTOR_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetModelProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkProperties.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <onboard_detector/GetStaticObstacles.h>
#include <thread>
#include <mutex>
#include <string>
#include <vector>

using std::cout; using std::endl;

namespace onboardDetector{
	
	struct StaticObstacleInfo {
		std::string name;
		uint32_t id;
		Eigen::Vector3d position;
		Eigen::Vector3d size;
		double yaw;
		std::string semantic_class;
		double cost_weight;
		double safety_distance;
	};

	class staticObstacleDetector{
	private:
		ros::NodeHandle nh_;
		ros::Timer updateTimer_;
		ros::Timer visTimer_;
		ros::Subscriber gazeboSub_;
		ros::Subscriber odomSub_;
		ros::Publisher visPub_;
		ros::ServiceServer getStaticObstacleServer_;

		// Gazebo services for detailed model information
		ros::ServiceClient getModelPropertiesClient_;
		ros::ServiceClient getModelStateClient_;
		ros::ServiceClient getLinkPropertiesClient_;

		// Parameters
		std::vector<std::string> excludeModels_;
		std::vector<std::string> includePatterns_;
		double detectionRange_;
		double defaultSafetyDistance_;
		double defaultCostWeight_;
		std::string robotFrame_;
		bool useDetailedGeometry_;
		bool firstTime_;

		// Data storage
		std::vector<StaticObstacleInfo> staticObstacles_;
		nav_msgs::Odometry robotOdom_;
		std::mutex obstaclesMutex_;

		// Visualization
		visualization_msgs::MarkerArray visMsg_;

	public:
		staticObstacleDetector(const ros::NodeHandle& nh);
		~staticObstacleDetector() = default;

		// Callbacks
		void gazeboModelStatesCB(const gazebo_msgs::ModelStatesConstPtr& allStates);
		void odomCB(const nav_msgs::OdometryConstPtr& odom);
		void updateTimerCB(const ros::TimerEvent&);
		void visTimerCB(const ros::TimerEvent&);

		// Service
		bool getStaticObstacles(onboard_detector::GetStaticObstacles::Request& req,
								onboard_detector::GetStaticObstacles::Response& res);

		// Core functions
		void processGazeboModels(const std::vector<std::string>& modelNames,
								const std::vector<geometry_msgs::Pose>& modelPoses);
		bool isStaticObstacle(const std::string& modelName);
		bool extractModelGeometry(const std::string& modelName, 
								  const geometry_msgs::Pose& modelPose,
								  StaticObstacleInfo& obstacle);
		Eigen::Vector3d estimateModelSize(const std::string& modelName);
		std::string classifyObstacle(const std::string& modelName);
		double getYawFromPose(const geometry_msgs::Pose& pose);

		// Filtering and range checking
		std::vector<StaticObstacleInfo> getObstaclesInRange(const Eigen::Vector3d& robotPos, double range);
		bool isInDetectionRange(const Eigen::Vector3d& obstaclePos, const Eigen::Vector3d& robotPos);

		// Visualization
		void updateVisualization();
		void publishStaticObstacles();

		// Utility functions
		void initializeGazeboServices();
		void loadParameters();
	};
}

#endif 