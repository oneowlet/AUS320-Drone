#ifndef __STM32_H__
#define __STM32_H__

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>

class Stm32_Data_t
{
private:
    
    ros::Subscriber stm32_sub;

    void feed(const std_msgs::StringConstPtr& pMsg);

public:

    ros::Publisher stm32_pub;
    std_msgs::String stm32_msg;
    serial::Serial Stm32_Serial; // 声明串口对象

    Stm32_Data_t(ros::NodeHandle& nh);
    ~Stm32_Data_t();
};

#endif // __STM32_H__