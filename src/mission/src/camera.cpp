#include "camera.h"

Camera_Data_t::Camera_Data_t(ros::NodeHandle& nh)
{   
    camera_pub = nh.advertise<diansai_msgs::AnimalCount>("/animal_num", 1);                                             
    camera_sub = nh.subscribe<yolov8_ros_msgs::BoundingBoxes>("/yolov8/BoundingBoxes", 1, boost::bind(&Camera_Data_t::feed, this, _1), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
}

Camera_Data_t::~Camera_Data_t()
{
    camera_pub.shutdown();
    camera_sub.shutdown();
    cv::destroyAllWindows();
}

void Camera_Data_t::feed(const yolov8_ros_msgs::BoundingBoxesConstPtr& pmsg)
{
    boxes_all_info = *pmsg;
}

void Camera_Data_t::animal_detection()
{
    // 检测阶段全局变量重置
    have_animal = false;  
    animal_info.bird = 0;
    animal_info.wolf = 0;
    animal_info.tiger = 0;
    animal_info.monkey = 0;
    animal_info.elephant = 0;
    boxes_square_info.clear();

    // 获取方块大小的半径内的动物信息
    if(boxes_all_info.bounding_boxes.size() == 0) return;
    else
    {
        for(int i = 0; i < boxes_all_info.bounding_boxes.size(); i++)
        {
            double distance_x = fabs(red_x - (boxes_all_info.bounding_boxes[i].xmax + boxes_all_info.bounding_boxes[i].xmin) / 2);
            double distance_y = fabs(red_y - (boxes_all_info.bounding_boxes[i].ymax + boxes_all_info.bounding_boxes[i].ymin) / 2);
            double distance = std::max(distance_x, distance_y);
            if(distance > in_square_threshold) continue;
            else
            {
                have_animal = true;
                boxes_square_info.push_back(boxes_all_info.bounding_boxes[i]);
                if(boxes_all_info.bounding_boxes[i].Class == "wolf") animal_info.wolf++;
                else if(boxes_all_info.bounding_boxes[i].Class == "peacock") animal_info.bird++;
                else if(boxes_all_info.bounding_boxes[i].Class == "tiger") animal_info.tiger++;
                else if(boxes_all_info.bounding_boxes[i].Class == "elephant") animal_info.elephant++;
                else if(boxes_all_info.bounding_boxes[i].Class == "monkey") animal_info.monkey++;
            }
        }
    }
    if(!have_animal) return; // 视野内有动物但是不在方格内
}

void Camera_Data_t::animal_tracking(const std::vector<yolov8_ros_msgs::BoundingBox>& boxes_in_square, const int& index)
{
    std::vector<yolov8_ros_msgs::BoundingBox> boxes;

    // 筛选出全视野内属于方格中的类的动物
    for(int i = 0; i < boxes_all_info.bounding_boxes.size(); i++)
    {
        for(int j = 0; j < boxes_in_square.size(); j++)
        {
            if(boxes_all_info.bounding_boxes[i].Class == boxes_in_square[j].Class)
            {
                boxes.push_back(boxes_all_info.bounding_boxes[i]);
                break;
            }
        }
    }

    // 数组如果越界直接返回防止运行错误
    if(index >= boxes.size()) 
    {   
        ROS_WARN("在追踪第%d只动物时丢帧", index + 1);
        return;
    }

    // 对全视野内属于方格中的类的动物进行排序(依据横轴像素坐标从小到大排序)
    std::sort(boxes.begin(), boxes.end(), [](const yolov8_ros_msgs::BoundingBox& a, const yolov8_ros_msgs::BoundingBox& b) { return a.xmin < b.xmin; });
    
    // 获取目标动物的中心像素坐标
    des_x = (boxes[index].xmax + boxes[index].xmin) / 2;
    des_y = (boxes[index].ymax + boxes[index].ymin) / 2;

    // 计算全局坐标系下的控制量
    cmd_dx = - (des_y - red_y) * pix2meter;
    cmd_dy = - (des_x - red_x) * pix2meter;
    ROS_INFO("x轴像素差: %d, y轴像素差: %d", - (des_y - red_y), - (des_x - red_x));
}

bool Camera_Data_t::check_arrival()
{
    return ( sqrt( pow(red_x - des_x, 2) + pow(red_y - des_y, 2) ) < arrive_pix );
}