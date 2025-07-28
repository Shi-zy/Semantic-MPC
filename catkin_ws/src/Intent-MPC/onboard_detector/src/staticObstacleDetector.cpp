/*
	FILE: staticObstacleDetector.cpp
	--------------------------------
	Implementation of static obstacle detector
*/

#include <onboard_detector/staticObstacleDetector.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/ColorRGBA.h>

namespace onboardDetector{

	staticObstacleDetector::staticObstacleDetector(const ros::NodeHandle& nh) : nh_(nh), firstTime_(true) {
		this->loadParameters();
		this->initializeGazeboServices();

		// Subscribers
		this->gazeboSub_ = this->nh_.subscribe("/gazebo/model_states", 10, 
			&staticObstacleDetector::gazeboModelStatesCB, this);
		
		std::string odomTopic;
		this->nh_.param<std::string>("static_detector/odom_topic", odomTopic, "/CERLAB/quadcopter/odom");
		this->odomSub_ = this->nh_.subscribe(odomTopic, 10, 
			&staticObstacleDetector::odomCB, this);

		// Publishers (simplified - using visualization only for now)
		// Note: Static obstacle array publisher removed to avoid circular dependency
		this->visPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(
			"static_detector/visualization", 10);

		// Service
		this->getStaticObstacleServer_ = this->nh_.advertiseService(
			"static_detector/get_static_obstacles", 
			&staticObstacleDetector::getStaticObstacles, this);

		// Timers
		this->updateTimer_ = this->nh_.createTimer(ros::Duration(1.0), 
			&staticObstacleDetector::updateTimerCB, this);
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), 
			&staticObstacleDetector::visTimerCB, this);

		ROS_INFO("[StaticObstacleDetector]: Initialized successfully");
	}

	void staticObstacleDetector::loadParameters() {
		// Detection range
		this->nh_.param<double>("static_detector/detection_range", this->detectionRange_, 20.0);
		
		// Safety parameters
		this->nh_.param<double>("static_detector/default_safety_distance", this->defaultSafetyDistance_, 0.5);
		this->nh_.param<double>("static_detector/default_cost_weight", this->defaultCostWeight_, 1.0);
		
		// Frame parameters
		this->nh_.param<std::string>("static_detector/robot_frame", this->robotFrame_, "map");
		this->nh_.param<bool>("static_detector/use_detailed_geometry", this->useDetailedGeometry_, false);

		// Exclude models (models to ignore) - 明确排除墙壁和走廊
		std::vector<std::string> defaultExclude = {"ground_plane", "quadcopter", "person", "dynamic_box", "dynamic_cylinder", "wall", "corridor", "building"};
		this->nh_.getParam("static_detector/exclude_models", this->excludeModels_);
		if (this->excludeModels_.empty()) {
			this->excludeModels_ = defaultExclude;
		}

		// Include patterns (model name patterns to include) - 排除墙壁，只检测box类障碍物
		std::vector<std::string> defaultInclude = {"box", "cylinder", "obstacle"};  // 移除 "wall", "corridor", "building"
		this->nh_.getParam("static_detector/include_patterns", this->includePatterns_);
		if (this->includePatterns_.empty()) {
			this->includePatterns_ = defaultInclude;
		}

		ROS_INFO("[StaticObstacleDetector]: Detection range: %.2f m", this->detectionRange_);
		ROS_INFO("[StaticObstacleDetector]: Safety distance: %.2f m", this->defaultSafetyDistance_);
		
		// 打印include和exclude模式
		ROS_INFO("[StaticObstacleDetector]: Include patterns:");
		for (const auto& pattern : this->includePatterns_) {
			ROS_INFO("  - %s", pattern.c_str());
		}
		ROS_INFO("[StaticObstacleDetector]: Exclude patterns:");
		for (const auto& pattern : this->excludeModels_) {
			ROS_INFO("  - %s", pattern.c_str());
		}
	}

	void staticObstacleDetector::initializeGazeboServices() {
		try {
			// Wait for Gazebo services with timeout
			if (ros::service::waitForService("/gazebo/get_model_properties", ros::Duration(5.0))) {
				this->getModelPropertiesClient_ = this->nh_.serviceClient<gazebo_msgs::GetModelProperties>("/gazebo/get_model_properties");
				ROS_INFO("[StaticObstacleDetector]: Connected to model properties service");
			}
			
			if (ros::service::waitForService("/gazebo/get_model_state", ros::Duration(5.0))) {
				this->getModelStateClient_ = this->nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
				ROS_INFO("[StaticObstacleDetector]: Connected to model state service");
			}
			
			if (ros::service::waitForService("/gazebo/get_link_properties", ros::Duration(5.0))) {
				this->getLinkPropertiesClient_ = this->nh_.serviceClient<gazebo_msgs::GetLinkProperties>("/gazebo/get_link_properties");
				ROS_INFO("[StaticObstacleDetector]: Connected to link properties service");
			}
		} catch (const ros::Exception& e) {
			ROS_WARN("[StaticObstacleDetector]: Could not connect to all Gazebo services: %s", e.what());
			ROS_WARN("[StaticObstacleDetector]: Will use basic model parsing only");
		}
	}

	void staticObstacleDetector::gazeboModelStatesCB(const gazebo_msgs::ModelStatesConstPtr& allStates) {
		if (this->firstTime_) {
			ROS_INFO("[StaticObstacleDetector]: Received first Gazebo model states");
			this->firstTime_ = false;
		}

		this->processGazeboModels(allStates->name, allStates->pose);
	}

	void staticObstacleDetector::odomCB(const nav_msgs::OdometryConstPtr& odom) {
		this->robotOdom_ = *odom;
	}

	void staticObstacleDetector::updateTimerCB(const ros::TimerEvent&) {
		this->publishStaticObstacles();
	}

	void staticObstacleDetector::visTimerCB(const ros::TimerEvent&) {
		this->updateVisualization();
	}

	void staticObstacleDetector::processGazeboModels(const std::vector<std::string>& modelNames,
													const std::vector<geometry_msgs::Pose>& modelPoses) {
		std::lock_guard<std::mutex> lock(this->obstaclesMutex_);
		this->staticObstacles_.clear();

		// ROS_INFO("[StaticObstacleDetector]: Processing %lu models from Gazebo", modelNames.size());
		
		for (size_t i = 0; i < modelNames.size(); ++i) {
			const std::string& modelName = modelNames[i];
			const geometry_msgs::Pose& modelPose = modelPoses[i];

			// ROS_INFO("[StaticObstacleDetector]: Model %lu: %s at (%.2f, %.2f, %.2f)", 
			//	i, modelName.c_str(), modelPose.position.x, modelPose.position.y, modelPose.position.z);

			if (this->isStaticObstacle(modelName)) {
				// ROS_INFO("[StaticObstacleDetector]: Model %s accepted as static obstacle", modelName.c_str());
				StaticObstacleInfo obstacle;
				if (this->extractModelGeometry(modelName, modelPose, obstacle)) {
					obstacle.id = static_cast<uint32_t>(this->staticObstacles_.size());
					this->staticObstacles_.push_back(obstacle);
					// ROS_INFO("[StaticObstacleDetector]: Added obstacle %s with size (%.2f, %.2f, %.2f)", 
					//	modelName.c_str(), obstacle.size(0), obstacle.size(1), obstacle.size(2));
				} else {
					ROS_WARN("[StaticObstacleDetector]: Failed to extract geometry for model %s", modelName.c_str());
				}
			} else {
				// ROS_INFO("[StaticObstacleDetector]: Model %s rejected", modelName.c_str());
			}
		}

		// 只在第一次检测到障碍物或数量明显变化时输出
		static size_t lastObstacleCount = 0;
		static bool firstDetection = true;
		if (firstDetection && this->staticObstacles_.size() > 0) {
			ROS_INFO("[StaticObstacleDetector]: First detection - found %lu static obstacles:", this->staticObstacles_.size());
			for (size_t i = 0; i < std::min(size_t(5), this->staticObstacles_.size()); ++i) {
				const auto& obs = this->staticObstacles_[i];
				ROS_INFO("  - %s: pos(%.3f, %.3f, %.3f), size(%.3f, %.3f, %.3f), yaw=%.3f", 
					obs.name.c_str(), obs.position(0), obs.position(1), obs.position(2),
					obs.size(0), obs.size(1), obs.size(2), obs.yaw);
			}
			if (this->staticObstacles_.size() > 5) {
				ROS_INFO("  - ... and %lu more obstacles", this->staticObstacles_.size() - 5);
			}
			firstDetection = false;
			lastObstacleCount = this->staticObstacles_.size();
		} else if (abs(static_cast<int>(this->staticObstacles_.size()) - static_cast<int>(lastObstacleCount)) > 2) {
			ROS_INFO("[StaticObstacleDetector]: Obstacle count changed from %lu to %lu", 
				lastObstacleCount, this->staticObstacles_.size());
			lastObstacleCount = this->staticObstacles_.size();
		}
	}

	bool staticObstacleDetector::isStaticObstacle(const std::string& modelName) {
		// Check exclude list
		for (const std::string& excludePattern : this->excludeModels_) {
			if (modelName.find(excludePattern) != std::string::npos) {
				// ROS_DEBUG("[StaticObstacleDetector]: Model %s excluded by pattern '%s'", 
				//	modelName.c_str(), excludePattern.c_str());
				return false;
			}
		}

		// Check include patterns
		for (const std::string& includePattern : this->includePatterns_) {
			if (modelName.find(includePattern) != std::string::npos) {
				// ROS_DEBUG("[StaticObstacleDetector]: Model %s included by pattern '%s'", 
				//	modelName.c_str(), includePattern.c_str());
				return true;
			}
		}

		// ROS_DEBUG("[StaticObstacleDetector]: Model %s not matching any include pattern", modelName.c_str());
		return false;
	}

	bool staticObstacleDetector::extractModelGeometry(const std::string& modelName, 
													 const geometry_msgs::Pose& modelPose,
													 StaticObstacleInfo& obstacle) {
		obstacle.name = modelName;
		obstacle.position = Eigen::Vector3d(modelPose.position.x, modelPose.position.y, modelPose.position.z);
		obstacle.yaw = this->getYawFromPose(modelPose);
		obstacle.semantic_class = this->classifyObstacle(modelName);
		obstacle.cost_weight = this->defaultCostWeight_;
		obstacle.safety_distance = this->defaultSafetyDistance_;

		// Try to get detailed geometry if enabled
		if (this->useDetailedGeometry_ && this->getModelPropertiesClient_.exists()) {
			// TODO: Implement detailed geometry extraction using Gazebo services
			obstacle.size = this->estimateModelSize(modelName);
		} else {
			obstacle.size = this->estimateModelSize(modelName);
		}

		// Basic validation
		if (obstacle.size.norm() < 0.01) {
			ROS_WARN("[StaticObstacleDetector]: Invalid size for model: %s", modelName.c_str());
			return false;
		}

		return true;
	}

	Eigen::Vector3d staticObstacleDetector::estimateModelSize(const std::string& modelName) {
		// Try to extract size from model name if it contains size information
		// Format examples: "wall_2_0.2_3", "box_1.5_1.5_2.0", etc.
		
		// Look for size patterns in the name
		std::vector<double> dimensions;
		std::string::size_type pos = 0;
		
		// Try to find numeric patterns
		while (pos < modelName.length()) {
			pos = modelName.find_first_of("0123456789.", pos);
			if (pos == std::string::npos) break;
			
			std::string::size_type end = modelName.find_first_not_of("0123456789.", pos);
			if (end == std::string::npos) end = modelName.length();
			
			std::string numStr = modelName.substr(pos, end - pos);
			try {
				double value = std::stod(numStr);
				if (value > 0.01 && value < 100.0) {  // Reasonable size range
					dimensions.push_back(value);
				}
			} catch (const std::exception&) {
				// Ignore invalid numbers
			}
			
			pos = end;
		}

		// Use extracted dimensions or defaults
		if (dimensions.size() >= 3) {
			return Eigen::Vector3d(dimensions[0], dimensions[1], dimensions[2]);
		} else if (dimensions.size() == 2) {
			return Eigen::Vector3d(dimensions[0], dimensions[1], 2.0);  // Default height
		} else if (dimensions.size() == 1) {
			return Eigen::Vector3d(dimensions[0], dimensions[0], 2.0);  // Square base
		}

		// Default sizes based on model type
		if (modelName.find("wall") != std::string::npos) {
			return Eigen::Vector3d(2.0, 0.2, 2.5);
		} else if (modelName.find("corridor") != std::string::npos) {
			return Eigen::Vector3d(10.0, 10.0, 2.5);
		} else if (modelName.find("building") != std::string::npos) {
			return Eigen::Vector3d(5.0, 5.0, 3.0);
		} else if (modelName.find("box") != std::string::npos) {
			return Eigen::Vector3d(1.0, 1.0, 1.0);
		} else if (modelName.find("cylinder") != std::string::npos) {
			return Eigen::Vector3d(1.0, 1.0, 2.0);
		}

		// Generic default
		return Eigen::Vector3d(1.0, 1.0, 1.0);
	}

	std::string staticObstacleDetector::classifyObstacle(const std::string& modelName) {
		if (modelName.find("wall") != std::string::npos) {
			return "WALL";
		} else if (modelName.find("building") != std::string::npos) {
			return "BUILDING";
		} else if (modelName.find("corridor") != std::string::npos) {
			return "CORRIDOR";
		} else if (modelName.find("furniture") != std::string::npos) {
			return "FURNITURE";
		} else {
			return "OBSTACLE";
		}
	}

	double staticObstacleDetector::getYawFromPose(const geometry_msgs::Pose& pose) {
		tf::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
		tf::Matrix3x3 mat(quat);
		double roll, pitch, yaw;
		mat.getRPY(roll, pitch, yaw);
		return yaw;
	}

	bool staticObstacleDetector::getStaticObstacles(onboard_detector::GetStaticObstacles::Request& req,
													onboard_detector::GetStaticObstacles::Response& res) {
		std::lock_guard<std::mutex> lock(this->obstaclesMutex_);

		Eigen::Vector3d robotPos(req.current_position.x, req.current_position.y, req.current_position.z);
		std::vector<StaticObstacleInfo> obstaclesInRange = this->getObstaclesInRange(robotPos, req.range);

		// Convert to ROS message format using arrays
		for (const StaticObstacleInfo& obsInfo : obstaclesInRange) {
			// Apply semantic filter if specified
			if (!req.semantic_filter.empty() && obsInfo.semantic_class != req.semantic_filter) {
				continue;
			}
			
			res.names.push_back(obsInfo.name);
			res.ids.push_back(obsInfo.id);
			
			geometry_msgs::Vector3 position;
			position.x = obsInfo.position(0);
			position.y = obsInfo.position(1);
			position.z = obsInfo.position(2);
			res.positions.push_back(position);
			
			geometry_msgs::Vector3 size;
			size.x = obsInfo.size(0);
			size.y = obsInfo.size(1);
			size.z = obsInfo.size(2);
			res.sizes.push_back(size);
			
			res.yaws.push_back(obsInfo.yaw);
			res.semantic_classes.push_back(obsInfo.semantic_class);
			res.cost_weights.push_back(obsInfo.cost_weight);
			res.safety_distances.push_back(obsInfo.safety_distance);
		}

		ROS_DEBUG("[StaticObstacleDetector]: Returned %lu obstacles in range %.2f m", 
				 res.positions.size(), req.range);
		return true;
	}

	std::vector<StaticObstacleInfo> 
	staticObstacleDetector::getObstaclesInRange(const Eigen::Vector3d& robotPos, double range) {
		std::vector<StaticObstacleInfo> result;
		
		for (const StaticObstacleInfo& obstacle : this->staticObstacles_) {
			if (this->isInDetectionRange(obstacle.position, robotPos)) {
				double distance = (obstacle.position - robotPos).norm();
				if (distance <= range) {
					result.push_back(obstacle);
				}
			}
		}

		// Sort by distance (closest first)
		std::sort(result.begin(), result.end(), 
			[&robotPos](const StaticObstacleInfo& a, const StaticObstacleInfo& b) {
				double distA = (a.position - robotPos).norm();
				double distB = (b.position - robotPos).norm();
				return distA < distB;
			});

		return result;
	}

	bool staticObstacleDetector::isInDetectionRange(const Eigen::Vector3d& obstaclePos, 
													const Eigen::Vector3d& robotPos) {
		double distance = (obstaclePos - robotPos).norm();
		return distance <= this->detectionRange_;
	}

	void staticObstacleDetector::updateVisualization() {
		std::lock_guard<std::mutex> lock(this->obstaclesMutex_);
		
		this->visMsg_.markers.clear();
		
		// 增加调试输出频率以诊断问题
		static int debugCounter = 0;
		if (++debugCounter % 10 == 0) {  // 每10次调用输出一次
			ROS_WARN("[StaticObstacleDetector]: Visualizing %lu obstacles", this->staticObstacles_.size());
			// 输出前几个障碍物的位置
			for (size_t i = 0; i < std::min(size_t(3), this->staticObstacles_.size()); ++i) {
				ROS_WARN("[StaticObstacleDetector]: Obstacle %lu: %s at pos(%.2f, %.2f, %.2f)", 
					i, this->staticObstacles_[i].name.c_str(),
					this->staticObstacles_[i].position(0), 
					this->staticObstacles_[i].position(1), 
					this->staticObstacles_[i].position(2));
			}
		}
		
		for (size_t i = 0; i < this->staticObstacles_.size(); ++i) {
			const StaticObstacleInfo& obstacle = this->staticObstacles_[i];
			
			visualization_msgs::Marker marker;
			marker.header.frame_id = "map";  // 固定使用map坐标系，与Gazebo世界坐标系一致
			marker.header.stamp = ros::Time::now();
			marker.ns = "static_obstacles";
			marker.id = static_cast<int>(i);
			marker.type = visualization_msgs::Marker::CUBE;
			marker.action = visualization_msgs::Marker::ADD;
			
			marker.pose.position.x = obstacle.position(0);
			marker.pose.position.y = obstacle.position(1);
			marker.pose.position.z = obstacle.position(2);
			
			tf::Quaternion quat;
			quat.setRPY(0, 0, obstacle.yaw);
			marker.pose.orientation.x = quat.x();
			marker.pose.orientation.y = quat.y();
			marker.pose.orientation.z = quat.z();
			marker.pose.orientation.w = quat.w();
			
			marker.scale.x = obstacle.size(0);
			marker.scale.y = obstacle.size(1);
			marker.scale.z = obstacle.size(2);
			
			// Color based on semantic class
			if (obstacle.semantic_class == "WALL") {
				marker.color.r = 0.5; marker.color.g = 0.5; marker.color.b = 0.5; marker.color.a = 0.6;
			} else if (obstacle.semantic_class == "BUILDING") {
				marker.color.r = 0.8; marker.color.g = 0.6; marker.color.b = 0.4; marker.color.a = 0.6;
			} else {
				marker.color.r = 0.6; marker.color.g = 0.3; marker.color.b = 0.3; marker.color.a = 0.6;
			}
			
			marker.lifetime = ros::Duration(0.5);
			this->visMsg_.markers.push_back(marker);
		}
		
		this->visPub_.publish(this->visMsg_);
	}

	void staticObstacleDetector::publishStaticObstacles() {
		// Simplified function - obstacle data now available via service only
		// This avoids circular dependency with map_manager messages
		ROS_DEBUG("[StaticObstacleDetector]: %lu obstacles available via service", this->staticObstacles_.size());
	}

} 