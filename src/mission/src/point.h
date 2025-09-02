#ifndef __WAYPOINT_H__
#define __WAYPOINT_H__

#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <diansai_msgs/WayPoint.h>
#include <geometry_msgs/Quaternion.h>
#include <diansai_msgs/ConfigPoint.h>
#include <quadrotor_msgs/TakeoffLand.h>

class Point_Data_t
{
private:
    
    ros::Subscriber point_sub;
    ros::Subscriber config_point_sub;

    void feed(const nav_msgs::OdometryConstPtr pMsg);
    void config_feed(const diansai_msgs::ConfigPointConstPtr pMsg);

public:

    ros::Publisher land_pub;
    ros::Publisher point_pub;

    int temp = 0;
    int goal_reached_cnt = 0;
    int config_points_index = 0;
    nav_msgs::Odometry current_point;
    diansai_msgs::WayPoint bias_point;
    diansai_msgs::WayPoint control_point;
    quadrotor_msgs::TakeoffLand land_msg;
    diansai_msgs::ConfigPoint config_points[100];

    Point_Data_t(ros::NodeHandle& nh);
    ~Point_Data_t();
    double fromQuaternion2yaw();
    double fromStr2double(const std::string& str);
    bool check_arrival(const diansai_msgs::WayPoint& point_control);
    
    diansai_msgs::WayPoint getBias();
    diansai_msgs::WayPoint ComputeLandingWithBias();
    diansai_msgs::WayPoint ComputeAnimalWithBias(const double& cmd_x, const double& cmd_y);
    diansai_msgs::WayPoint ComputeTargetWithBias(const diansai_msgs::ConfigPoint& point_config);
};

#endif // __WAYPOINT_H__