#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <cood_tran_msgs/location.h>
#include <cood_tran_msgs/car_interact.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h> //点云头文件,用于发布接受pcl对象
#include <pcl_conversions/pcl_conversions.h> //点云转换头文件
#include <sensor_msgs/PointCloud2.h>         //消息头文件
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <math.h>
#include "ros/ros.h"

std::string target_obj;
int u,v;
float camera_x,camera_y,camera_z;
int get_target;
void darknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);
void pointCouldCallback( const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg);
void findObjCallback(const std_msgs::Int8::ConstPtr &msg);
bool location(cood_tran_msgs::location::Request &req,cood_tran_msgs::location::Response &res);

int main(int argc, char **argv) 
{

  ros::init(argc, argv, "cood_tran");
  ros::NodeHandle nh;

  ros::param::get("target", target_obj);
  ros::Subscriber ros_coord_pixel_sub =
  nh.subscribe("/darknet_ros/bounding_boxes", 1, &darknetCallback);

  ros::Subscriber find_obj_sub =
  nh.subscribe("/darknet_ros/found_object", 1, &findObjCallback);

  ros::Subscriber point_cloud_sub =
  nh.subscribe("camera/depth_registered/points", 1, &pointCouldCallback);///camera/depth_registered/points  /camera/depth_registered/points<->color_optical
    
  ros::ServiceServer location_server =
  nh.advertiseService("location_srv", &location);




  ros::spin();
  return 0;
}

//查找目标物体 得到像素坐标
    void darknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
    {
    std::cout << "Find something ! " ;
    std::cout << "Boundingbox size " << sizeof(msg->bounding_boxes) << std::endl;//!!!!!!!!!!!!!!!!!!!!!!!!
    for (size_t i = 0;i < sizeof(msg->bounding_boxes);i++)
    {
        std::cout << " class "  << msg->bounding_boxes[i].Class<< std::endl;
        if (msg->bounding_boxes[i].Class == target_obj)
        {
            std::cout << " and it is  target ! "  << std::endl;
            u = (msg->bounding_boxes[i].xmin + msg->bounding_boxes[i].xmax) / 2;
            v = (msg->bounding_boxes[i].ymin + msg->bounding_boxes[i].ymax) / 2;
            std::cout << " dark_u " << u << "dark_v" <<v <<std::endl;
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

    //订阅识别物体数量 get_target 清零
    void findObjCallback(const std_msgs::Int8::ConstPtr &msg)
    {
    if (msg->data == 0)
    {
        std::cout << " Find nothing ! " << std::endl;
        std::cout << "\033[2J\033[1;1H";     // clear terminal
        get_target = 0;
    }
    }

    //将图像坐标转换为相对于相机的坐标
    void pointCouldCallback( const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg) 
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
                std::cout << " point_u " << u << "point_v" <<v <<std::endl;
                pcl::PointXYZ pt = point_pcl.at(u,v);
                //旧版本的realsense包乘以0.124987系数 单位mm
                camera_x = pt.x;
                camera_y = pt.y;
                camera_z = pt.z;
                std::cout << " coordnate get " << std::endl;
            
            }
            else
            std::cout << " the pointcloud is not organized " << std::endl;
            //std::cout << "\033[2J\033[1;1H";     // clear terminal
        }

    }

    //将相对于相机的坐标转换为相对于机械臂的坐标
    bool location(cood_tran_msgs::location::Request &req,
                cood_tran_msgs::location::Response &res) 
    {
        res.get_target = get_target;
        if(get_target ==1)
        {
            if(!std::isnan(camera_z))
            {
                //乘以转换矩阵得到物体相对于机械臂的坐标
                float final_x, final_y, final_z;
                final_x = camera_x;
                final_y = camera_y;
                final_z = camera_z;
                
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