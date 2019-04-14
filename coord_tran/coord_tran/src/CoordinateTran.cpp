#include "ros/ros.h"
#include "CoordinateTran.h"


namespace huskybot_arm
{ 

    CoordinateTran::CoordinateTran(ros::NodeHandle nh)
        : nh(nh),get_target(0)
    {
    ros::param::get("target", target_obj);

    std::cout << "target_obj_class" << target_obj <<std::endl;

    ROS_INFO("[CoordinateTran] Node started.");

    find_obj_sub =
    nh.subscribe("/darknet_ros/found_object", 1, &CoordinateTran::findObjCallback,this);


    ros_coord_pixel_sub =
    nh.subscribe("/darknet_ros/bounding_boxes", 1, &CoordinateTran::darknetCallback,this);

   
    point_cloud_sub =
    nh.subscribe("/camera/depth_registered/points", 1, &CoordinateTran::pointCouldCallback,this);///camera/depth_registered/points  /camera/depth_registered/points<->color_optical
    
    location_server =
    nh.advertiseService("location_srv", &CoordinateTran::location,this);
    }


    //订阅识别物体数量 get_target 清零
    void CoordinateTran::findObjCallback(const std_msgs::Int8::ConstPtr &msg)
    {
    if (msg->data == 0)
    {
        std::cout << " Find nothing ! " << std::endl;
        std::cout << "\033[2J\033[1;1H";     // clear terminal
        get_target = 0;
    }
    }





    //查找目标物体 得到像素坐标
    void CoordinateTran::darknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
    {
    std::cout << "Find something ! " ;
    
    for (size_t i = 0;i < sizeof(msg->bounding_boxes);i++)
    {
        if (msg->bounding_boxes[i].Class == target_obj)
        {
            std::cout << " and it is  target ! "  << std::endl;
            u = (msg->bounding_boxes[i].xmin + msg->bounding_boxes[i].xmax) / 2;
            v = (msg->bounding_boxes[i].ymin + msg->bounding_boxes[i].ymax) / 2;
            std::cout << " dark_u "  << u << " dark_v "  << v << std::endl;
            get_target =1;
            break;
        }
        else
        {
            std::cout << " but it is not target ! "  << std::endl;
            get_target =0;
        }
    }
    // std::cout << "\033[2J\033[1;1H";     // clear terminal
    }



    //将图像坐标转换为相对于相机的坐标
    void CoordinateTran::pointCouldCallback( const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg) 
    {
    // #if pointCouldDebug
    //   std::cout << "pointCloud2 (header):" << point_cloud_msg->header << std::endl;
    // #endif

        if (get_target == 1)
        {
            pcl::PointCloud<pcl::PointXYZ> point_pcl;
            pcl::fromROSMsg(*point_cloud_msg, point_pcl);
            if (point_pcl.isOrganized ())
            {
                std::cout << " cloud_u "  << u << " cloud_v "  << v << std::endl;
                pcl::PointXYZ pt = point_pcl.at(u,v);
                //旧版本的realsense包乘以0.124987系数 单位m
                camera_x = pt.x;
                camera_y = pt.y;
                camera_z = pt.z;
                std::cout << " coordnate get: " << " camera_x " <<camera_x <<" camera_y " <<camera_y <<" camera_z " <<camera_z <<std::endl;
                
            
            }
            else
            std::cout << " the pointcloud is not organized " << std::endl;
            //std::cout << "\033[2J\033[1;1H";     // clear terminal
        }

    }

    //将相对于相机的坐标转换为相对于机械臂的坐标
    bool CoordinateTran::location(coord_tran_msgs::location::Request &req,
                coord_tran_msgs::location::Response &res) 
    {
        res.get_target = get_target;
        if(get_target ==1)
        {
            if(!std::isnan(camera_z))
            {
                //乘以转换矩阵得到物体相对于机械臂的坐标 单位米  

                float final_x, final_y, final_z;
                

                //最终坐标=模型转换坐标+牌照位置+修正值

                final_x = -camera_y+0.0705+0.20+0.01;
                final_y = -camera_x-0.0027+0.00+0.01;
                final_z = -camera_z+0.1035+0.07-0.005;
                
                res.x = final_x;
                res.y = final_y;
                res.z = final_z;
            }
            else
            {
                std::cout << "bad value "  <<std::endl;
            }
        }
        return true;
    }

}