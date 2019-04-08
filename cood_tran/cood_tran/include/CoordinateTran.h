#ifndef COORDINATETRAN_H
#define COORDINATETRAN_H

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

namespace huskybot_arm
{

  class CoordinateTran
  {

    public:
    //构造函数
    explicit CoordinateTran(ros::NodeHandle nh);



    private:
    ros::NodeHandle nh;
    std::string target_obj;
    int u,v;
    float camera_x,camera_y,camera_z;
    int get_target;

    ros::Subscriber ros_coord_pixel_sub;

    ros::Subscriber find_obj_sub ;

    ros::Subscriber point_cloud_sub;///camera/depth_registered/points  /camera/depth_registered/points<->color_optical
    
    ros::ServiceServer location_server;

    void findObjCallback(const std_msgs::Int8::ConstPtr &msg);

    void darknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);
    
    void pointCouldCallback( const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg);

    bool location(cood_tran_msgs::location::Request &req,cood_tran_msgs::location::Response &res);

    


    
  };


}

#endif
