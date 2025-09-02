#include "point.h"

Point_Data_t::Point_Data_t(ros::NodeHandle& nh)
{
    // 发布自定义航点消息
    point_pub = nh.advertise<diansai_msgs::WayPoint>("/px4ctrl/custom_waypoint", 1);
    // 发布起降消息
    land_pub = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 100); 
    // 订阅高频率里程计消息
    point_sub = nh.subscribe<nav_msgs::Odometry>("/odom_high_freq", 100, boost::bind(&Point_Data_t::feed, this, _1), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    // 订阅配置航点消息
    config_point_sub = nh.subscribe<diansai_msgs::ConfigPoint>("/config_point", 100, boost::bind(&Point_Data_t::config_feed, this, _1), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
}

Point_Data_t::~Point_Data_t()
{
    land_pub.shutdown(); // 释放着陆发布者
    point_pub.shutdown(); // 释放点发布者
    point_sub.shutdown(); // 释放点订阅者
}

void Point_Data_t::feed(const nav_msgs::OdometryConstPtr pMsg)
{
    current_point = *pMsg;
}

void Point_Data_t::config_feed(const diansai_msgs::ConfigPointConstPtr pMsg)
{
    config_points[config_points_index] = *pMsg;
    config_points_index++;
}

bool Point_Data_t::check_arrival(const diansai_msgs::WayPoint& point_control)
{
    double distance = sqrt(pow(current_point.pose.pose.position.x - point_control.x, 2) +
                           pow(current_point.pose.pose.position.y - point_control.y, 2) +
                           pow(current_point.pose.pose.position.z - point_control.z, 2));
    return distance < 0.20; // 阈值可调
}

diansai_msgs::WayPoint Point_Data_t::getBias()
{
    diansai_msgs::WayPoint bias;
    bias.x = current_point.pose.pose.position.x;
    bias.y = current_point.pose.pose.position.y;
    bias.z = current_point.pose.pose.position.z;
    bias.yaw = fromQuaternion2yaw();
    ROS_INFO("已记录零偏!");
    ROS_INFO("零偏: x = %f, y = %f, z = %f, yaw = %f", bias.x, bias.y, bias.z, bias.yaw);
    return bias; 
}

diansai_msgs::WayPoint Point_Data_t::ComputeTargetWithBias(const diansai_msgs::ConfigPoint& point_config)
{
    diansai_msgs::WayPoint point;
    point.x = fromStr2double(point_config.row) + bias_point.x;
    point.y = fromStr2double(point_config.col) + bias_point.y;
    point.z = bias_point.z + 1.0;
    point.yaw = bias_point.yaw;
    return point; 
}

diansai_msgs::WayPoint Point_Data_t::ComputeAnimalWithBias(const double& cmd_x, const double& cmd_y)
{
    diansai_msgs::WayPoint point;
    point.x = cmd_x + current_point.pose.pose.position.x;
    point.y = cmd_y + current_point.pose.pose.position.y;
    point.z = bias_point.z + 1.0;
    point.yaw = bias_point.yaw;
    return point;
}

diansai_msgs::WayPoint Point_Data_t::ComputeLandingWithBias()
{
    diansai_msgs::WayPoint point;
    point.x =  bias_point.x;
    point.y =   bias_point.y;
    point.z = bias_point.z;
    point.yaw = bias_point.yaw;
    return point; 
}

double Point_Data_t::fromQuaternion2yaw()
{
  geometry_msgs::Quaternion q = current_point.pose.pose.orientation;  
  double yaw = atan2( 2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z );
  return yaw * 180.0 / M_PI; // 返回角度值
}

double Point_Data_t::fromStr2double(const std::string& str)
{
    if(str == "A9") return 0.0;
    else if(str == "A8") return 0.5;
    else if(str == "A7") return 1.0;
    else if(str == "A6") return 1.5;
    else if(str == "A5") return 2.0;
    else if(str == "A4") return 2.5;
    else if(str == "A3") return 3.0; 
    else if(str == "A2") return 3.5;
    else if(str == "A1") return 4.0;
    else if(str == "B7") return 3.0;
    else if(str == "B6") return 2.5;
    else if(str == "B5") return 2.0;
    else if(str == "B4") return 1.5;
    else if(str == "B3") return 1.0;
    else if(str == "B2") return 0.5;
    else if(str == "B1") return 0.0;
    else return 0.0;
}