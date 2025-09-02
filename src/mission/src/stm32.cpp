#include "stm32.h"

Stm32_Data_t::Stm32_Data_t(ros::NodeHandle& nh)
{
    try
    {
        Stm32_Serial.setPort("/dev/stm32"); // udev规则设置的串口名称
        Stm32_Serial.setBaudrate(115200); // 波特率
        serial::Timeout to = serial::Timeout::simpleTimeout(2000); // 超时等
        Stm32_Serial.setTimeout(to); 
        Stm32_Serial.setBytesize(serial::eightbits); // 8个数据位
        Stm32_Serial.setParity(serial::parity_none); // 无奇偶校验位
        Stm32_Serial.setStopbits(serial::stopbits_one);     // 1个停止位
        Stm32_Serial.open(); // 串口开启
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR("串口打开失败!");
    }
    if(Stm32_Serial.isOpen()) { ROS_INFO("串口已打开!"); }

    stm32_pub = nh.advertise<std_msgs::String>("/stm32_cmd", 10);

    stm32_sub = nh.subscribe<std_msgs::String>("/stm32_cmd", 
                                                10,
                                                boost::bind(&Stm32_Data_t::feed, this, _1), 
                                                ros::VoidConstPtr(), 
                                                ros::TransportHints().tcpNoDelay());
}

Stm32_Data_t::~Stm32_Data_t()
{
    if(Stm32_Serial.isOpen())
    {
        Stm32_Serial.close(); // 关闭串口
        ROS_INFO("串口已关闭!");
    }
    stm32_pub.shutdown(); // 释放发布者
    stm32_sub.shutdown(); // 释放订阅者
}

void Stm32_Data_t::feed(const std_msgs::StringConstPtr& pMsg)
{
    int value = std::stoi(pMsg->data);
    if (value < 0 || value > 255)
    {
        ROS_WARN("接收到的数据超出有效范围: %d", value);
        return;
    }
    uint8_t byte = static_cast<uint8_t>(value);
    Stm32_Serial.write(&byte, 1);
}