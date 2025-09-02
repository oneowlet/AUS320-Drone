#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// 全局变量
Mat current_image;
string windowName = "点击图像查看像素坐标 (ESC退出)";
bool image_available = false;

// 鼠标事件回调函数
void onMouse(int event, int x, int y, int flags, void* userdata)
{
    // 当鼠标左键点击且有图像可用时
    if (event == EVENT_LBUTTONDOWN && image_available)
    {
        // 显示坐标信息
        ROS_INFO("像素坐标: (%d, %d)", x, y);
        
        // 在图像上绘制点和坐标文本
        Mat temp = current_image.clone();
        circle(temp, Point(x, y), 5, Scalar(0, 0, 255), -1); // 绘制红色点
        string text = "(" + to_string(x) + ", " + to_string(y) + ")";
        putText(temp, text, Point(x + 10, y - 10), 
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
        
        // 显示更新后的图像
        imshow(windowName, temp);
    }
}

// 图像回调函数
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // 将ROS图像消息转换为OpenCV格式
        current_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
        image_available = true;
        imshow(windowName, current_image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge转换错误: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "image_coordinate_node");
    ros::NodeHandle nh;
    
    // 创建窗口并设置鼠标回调
    namedWindow(windowName, WINDOW_AUTOSIZE);
    setMouseCallback(windowName, onMouse, 0);
    
    // 订阅图像话题
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageCallback);
    
    ROS_INFO("请在图像上点击查看像素坐标,按ESC键退出");
    
    // 处理窗口事件和ROS回调
    while (ros::ok())
    {
        // 处理窗口事件
        int key = waitKey(1);
        if (key == 27) // ESC键退出
        {
            break;
        }
        
        // 处理ROS回调
        ros::spinOnce();
    }
    
    // 清理资源
    destroyAllWindows();
    return 0;
}