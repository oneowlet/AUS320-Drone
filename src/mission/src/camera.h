#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <vector>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <diansai_msgs/AnimalCount.h>
#include <yolov8_ros_msgs/BoundingBox.h>
#include <yolov8_ros_msgs/BoundingBoxes.h>

class Camera_Data_t
{
private:

    ros::Subscriber camera_sub;

    void feed(const yolov8_ros_msgs::BoundingBoxesConstPtr& pmsg);

public:

    ros::Publisher camera_pub;

    int red_x = 340; // 激光笔像素坐标(恒定)
    int red_y = 235; // 激光笔像素坐标(恒定)
    int des_x = 0; // 目标动物的像素坐标(每次视觉伺服刷新)
    int des_y = 0; // 目标动物的像素坐标(每次视觉伺服刷洗)
    int picture_cnt = 0; // 是否是正确帧计数器(每次检测方格刷新)
    int arrive_cnt = 0; // 是否视觉伺服到达计数器(每次视觉伺服刷新)
    int arrive_pix = 50; // 视觉伺服已经到达的像素阈值(恒定)
    double cmd_dx = 0.0; // 视觉伺服坐标(每次视觉伺服刷新)
    double cmd_dy = 0.0; // 视觉伺服坐标(每次视觉伺服刷新)
    bool have_animal = false; // 判断方格内是否有动物(每次检测方格刷新)
    double pix2meter = 0.0024; // 视觉伺服的控制比例(恒定)
    double in_square_threshold = 150.0; // 方格像素大小半径阈值(恒定)
    diansai_msgs::AnimalCount animal_info; // 发送给地面站的动物信息(每次检测方格刷新)
    yolov8_ros_msgs::BoundingBoxes boxes_all_info; // 当前视野内的所有动物信息(回调刷新)
    std::vector<yolov8_ros_msgs::BoundingBox> boxes_square_info; // 当前方格内的动物信息(每次检测方格刷新)

    Camera_Data_t(ros::NodeHandle& nh);
    ~Camera_Data_t();
    bool check_arrival();
    void animal_detection();
    void animal_tracking(const std::vector<yolov8_ros_msgs::BoundingBox>& boxes_in_square, const int& index);
};

#endif // __CAMERA_H__