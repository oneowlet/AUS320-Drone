#include "point.h"
#include "stm32.h"
#include "camera.h"

int main(int argc, char  *argv[])
{
    // Step0 初始化设置
    setlocale(LC_ALL,""); // 设置中文环境,避免中文乱码
    ros::init(argc,argv,"main_node"); // 节点名称
    ros::NodeHandle nh("~"); // 私有命名空间
    
    Point_Data_t point_data(nh);
    Stm32_Data_t stm32_data(nh);
    Camera_Data_t camera_data(nh);
    
    ros::Rate r(10); // 10Hz
    ros::Duration(3.0).sleep(); // 等待3秒钟



    // Step1 记录零偏位姿
    ros::spinOnce();
    point_data.getBias();
    ROS_INFO("已记录零偏位姿!\n");



    // step2 准备就绪示意
    stm32_data.stm32_msg.data = "50";
    stm32_data.stm32_pub.publish(stm32_data.stm32_msg);
    ros::Duration(3.0).sleep(); // 等待3秒钟
    ros::spinOnce();
    ROS_INFO("准备就绪,正在等待地面站完成路径规划...\n");



    // Step3 存取规划航点
    while(ros::ok())
    {
        point_data.temp = point_data.config_points_index;
        ros::spinOnce();
        if(point_data.temp == point_data.config_points_index
           && point_data.config_points_index != 0)
        {
            ROS_INFO("路径规划已完成! 获取总规划航点数为：%d\n", point_data.config_points_index);
            break;
        }
        r.sleep(); // 50Hz
    }



    // Step4 巡航执行任务
    for(int i = 0; i < point_data.config_points_index; i++)
    {
        // 发布目标航点
        point_data.control_point = point_data.ComputeTargetWithBias(point_data.config_points[i]);
        point_data.point_pub.publish(point_data.control_point);
        ROS_INFO("请前往方格点(%s, %s)", point_data.config_points[i].col.c_str(), point_data.config_points[i].row.c_str()); 

        // 检查是否到达
        ros::Time start_time = ros::Time::now();
        while(ros::ok())
        {
            if(i != 0)
            {
                if( (ros::Time::now() - start_time).toSec() > 7.0 ) break;
            }
            ros::spinOnce();
            if (point_data.check_arrival(point_data.control_point)) 
            { 
                point_data.goal_reached_cnt++; 
            }
            if (point_data.goal_reached_cnt >= 5) 
            {
                point_data.goal_reached_cnt = 0;
                ROS_INFO("已到达第%d个目标方格点!", i + 1);
                break;
            }
            r.sleep(); // 10Hz
        }

        // 检查是否扫描
        if(!point_data.config_points[i].is_scan)
        {
            ROS_INFO("该方格点已扫描过,无须再次进行扫描!\n");
            continue;
        }
        else
        {
            ROS_INFO("该方格点未扫描过,开始进行扫描...");

            // 确保获取当前方格内正确的一帧图像
            while (ros::ok())
            {
                int temp_num = 0;
                if(camera_data.picture_cnt >= 5)
                { 
                    camera_data.picture_cnt = 0; 
                    break; 
                }
                temp_num = camera_data.boxes_all_info.bounding_boxes.size();
                ros::spinOnce();
                if(temp_num == camera_data.boxes_all_info.bounding_boxes.size()) 
                { 
                    camera_data.picture_cnt++; 
                }
                r.sleep(); // 10Hz
            }
            
            // 获取本次方格内的动物信息
            camera_data.animal_detection();
            if(!camera_data.have_animal) // 本次方格没有动物就继续巡航
            {
                ROS_INFO("该方格点内没有动物!\n");
                continue; 
            }
            ROS_INFO("该方格点内共有%ld动物!", camera_data.boxes_square_info.size()); 

            // 发送给地面站该方格内的动物信息
            camera_data.animal_info.col = point_data.config_points[i].col;
            camera_data.animal_info.row = point_data.config_points[i].row;
            camera_data.camera_pub.publish(camera_data.animal_info);
            ROS_INFO("已向地面站发送该方格点内动物信息!");
            
            // 方格点内动物跟踪处理
            for(int j = 0; j < camera_data.boxes_square_info.size(); j++)
            {   
                start_time = ros::Time::now();
                ROS_INFO("正在前往方格内第%d只动物位置!", j);
                // 当前下标下动物的视觉伺服
                while(ros::ok())
                {
                    if( (ros::Time::now() - start_time).toSec() > 10.0 ) break;
                    ros::spinOnce();
                    if(camera_data.check_arrival())
                    {
                        camera_data.arrive_cnt++;
                    }
                    if(camera_data.arrive_cnt >= 5)
                    {
                        camera_data.arrive_cnt = 0;
                        ROS_INFO("已到达方格内第%d只动物位置!", j + 1);
                        break;
                    }

                    // 计算出控制量坐标
                    camera_data.animal_tracking(camera_data.boxes_square_info, j); 
                    point_data.control_point = point_data.ComputeAnimalWithBias(camera_data.cmd_dx, camera_data.cmd_dy);
                    point_data.point_pub.publish(point_data.control_point);
                    r.sleep(); // 10Hz
                }
            }
            ROS_INFO("该方格点扫描完成!\n");
        }
    }



    // Step5 着陆并且停机
    point_data.control_point = point_data.ComputeLandingWithBias();
    point_data.point_pub.publish(point_data.control_point);
    ROS_INFO("正在着陆...\n");
    stm32_data.stm32_pub.publish(stm32_data.stm32_msg);
    ROS_INFO("指示灯闪烁!\n");
    while(ros::ok())
    {
        ros::spinOnce();
        if (point_data.check_arrival(point_data.control_point)) 
        { 
            point_data.goal_reached_cnt++; 
        }
        if (point_data.goal_reached_cnt >= 5) 
        {
            point_data.goal_reached_cnt = 0;
            ROS_INFO("已成功着陆!\n");
            break;
        }
        r.sleep(); // 10Hz
    }
    point_data.land_msg.takeoff_land_cmd = point_data.land_msg.LAND;
    point_data.land_pub.publish(point_data.land_msg);
    ROS_INFO("已停机!\n");

    

    // Step6 任务圆满完成
    ros::shutdown();
    ROS_INFO("任务圆满完成!");
    return 0;
}