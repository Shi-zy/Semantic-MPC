#include <ros/ros.h>
#include <onboard_detector/GetStaticObstacles.h>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_static_detector");
    ros::NodeHandle nh;
    
    // 等待服务可用
    ros::service::waitForService("/static_detector/get_static_obstacles", 10.0);
    
    // 创建服务客户端
    ros::ServiceClient client = nh.serviceClient<onboard_detector::GetStaticObstacles>("/static_detector/get_static_obstacles");
    
    // 准备服务请求
    onboard_detector::GetStaticObstacles srv;
    srv.request.current_position.x = 0.0;
    srv.request.current_position.y = 0.0;
    srv.request.current_position.z = 1.0;
    srv.request.range = 20.0;
    srv.request.semantic_filter = "";
    
    ROS_INFO("[Test]: Calling static obstacle detector service...");
    
    if (client.call(srv))
    {
        ROS_INFO("[Test]: Service call SUCCESS");
        ROS_INFO("[Test]: Detected %lu static obstacles", srv.response.positions.size());
        
        for (size_t i = 0; i < srv.response.positions.size(); ++i)
        {
            ROS_INFO("  Obstacle %lu: pos(%.2f, %.2f, %.2f), size(%.2f, %.2f, %.2f), yaw=%.2f",
                i,
                srv.response.positions[i].x,
                srv.response.positions[i].y,
                srv.response.positions[i].z,
                srv.response.sizes[i].x,
                srv.response.sizes[i].y,
                srv.response.sizes[i].z,
                srv.response.yaws[i]
            );
        }
    }
    else
    {
        ROS_ERROR("[Test]: Service call FAILED");
        return 1;
    }
    
    return 0;
} 