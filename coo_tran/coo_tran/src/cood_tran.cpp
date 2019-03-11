#include "ros/ros.h"
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
//#include <librealsense2/rs.hpp>

#include <pcl/point_cloud.h>//点云头文件,用于发布接受pcl对象
#include <pcl_conversions/pcl_conversions.h>//点云转换头文件
#include <sensor_msgs/PointCloud2.h>//消息头文件
#include <location_srv/Location.h>
#include <pcl/io/pcd_io.h>


int u,v;
double camera_x,camera_y,camera_z;



void darknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)

{
    // cout<<"Bouding Boxes (header):" << msg->header <<endl;
	// cout<<"Bouding Boxes (image_header):" << msg->image_header <<endl;
	// cout<<"Bouding Boxes (Class):" << msg->bounding_boxes[0].Class <<endl;
    u=(msg->bounding_boxes[0].xmin + msg->bounding_boxes[0].xmax)/2;
    v=(msg->bounding_boxes[0].ymin + msg->bounding_boxes[0].ymax)/2;
    // std::cout << "x" << x << std::endl;
    // std::cout << "y" << y << std::endl;
    //cout << "\033[2J\033[1;1H";     // clear terminal
}

void pointCouldCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
  //cout<<"pointCloud2 (header):" << point_cloud_msg->header <<endl;
  std::cout << "x" << u << std::endl;
  std::cout << "y" << v << std::endl;
  int pcl_index;
  //std::cout << "msg_height = " << point_cloud_msg->height << std::endl;
  
  pcl::PointCloud<pcl::PointXYZ> point_pcl;
  pcl::fromROSMsg(*point_cloud_msg, point_pcl);
  pcl_index = (v*480) + u; 
  std::cout<< "pcl_index" << pcl_index <<std::endl;
  std::cout << "(x,y,z) = " << point_pcl.at(pcl_index) << std::endl;
 
  
  std::cout << "cloud: width = " << point_pcl.width << " height = "<<point_pcl.height << std::endl;
  
//   BOOST_FOREACH (const pcl::PointXYZ& pt, point_pcl.points)
//   //std::cout << typeid( point_pcl.at(pcl_index)  ).name() <<std::endl;N3pcl8PointXYZE
//   //std::cout << "\033[2J\033[1;1H";
  
//   {
    
//       camera_x = pt.x;
//       camera_y = pt.y;
//       camera_z = pt.z;
//       printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
//       std::cout << "x  = " << camera_x <<" y = "<< camera_y <<" z = " <<camera_z<< std::endl;
//    }
  
}

bool location(location_srv::Location::Request &req,location_srv::Location::Response &res)
{
    //乘以转换矩阵得到final_x final_y final_z
    // res.x = final_x;
    // res.y = final_y;
    // res.z = final_z;
    std::cout << "x  = " << camera_x <<" y = "<< camera_y <<" z = " <<camera_z<< std::endl;
    res.x = 200;
    res.y = 200;
    res.z = 200;
    return true;
}



    int main(int argc, char **argv)
    {
        //初始化ROS
        ros::init(argc,argv,"cood_tran");
        ros::NodeHandle nh;
        ros::ServiceServer location_server = nh.advertiseService("location_srv",location);
        ROS_INFO("ready srv server");
        //创建接收darknet结果和点云信息的接受者
        ros::Subscriber ros_coord_pixel_sub = nh.subscribe("/darknet_ros/bounding_boxes",1,darknetCallback);
        ros::Subscriber point_cloud_sub = nh.subscribe("/camera/depth/color/points", 1, pointCouldCallback);
        

       //循环等待回调函数
        ros::spin();
        return 0;
    }


