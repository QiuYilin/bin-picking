#include "ros/ros.h"
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
//#include <librealsense2/rs.hpp>

#include <location_srv/Location.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h> //点云头文件,用于发布接受pcl对象
#include <pcl_conversions/pcl_conversions.h> //点云转换头文件
#include <sensor_msgs/PointCloud2.h>         //消息头文件
#define pointCouldDebug 0
#define coordinateDebug 0

int u, v;
float camera_x, camera_y, camera_z;
std::string obj_class;

void darknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)

{
  ROS_INFO("darknetCallback");
  obj_class = msg->bounding_boxes[0].Class;
  
  u = (msg->bounding_boxes[0].xmin + msg->bounding_boxes[0].xmax) / 2;
  v = (msg->bounding_boxes[0].ymin + msg->bounding_boxes[0].ymax) / 2;
  #if coordinateDebug
  std::cout << "u0 " << u << std::endl;
  std::cout << "v0 " << v << std::endl;
  #endif
  // boundingBoxesResults_.bounding_boxes.clear();
  // cout << "\033[2J\033[1;1H";     // clear terminal
}



void pointCouldCallback( const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg) {
     ROS_INFO("pointCouldCallback");
#if pointCouldDebug
  std::cout << "pointCloud2 (header):" << point_cloud_msg->header << std::endl;
#endif
#if coordinateDebug
  std::cout << "u " << u << std::endl;
  std::cout << "v " << v << std::endl;
#endif
  pcl::PointCloud<pcl::PointXYZ> point_pcl;
  pcl::fromROSMsg(*point_cloud_msg, point_pcl);
#if pointCloudDebug
  std::cout << "cloud: width = " << point_pcl.width
            << " height = " << point_pcl.height << std::endl;
#endif
  auto pt = point_pcl.at(u,v);
  
  camera_x = pt.x;
  camera_y = pt.y;
  camera_z = 0.1 * pt.z;
  
}



bool location(location_srv::Location::Request &req,
              location_srv::Location::Response &res) {
  obj_class = "None";
  std::cout << "obj_class " << obj_class <<std::endl;
  if (obj_class != "napkin1")
    res.arm_on = 0;
  else {
    std::cout << "Target get."<< std::endl;
    res.arm_on = 1;
    //乘以转换矩阵得到final_x final_y final_z
    float final_x, final_y, final_z;
    final_x = camera_x;
    final_y = camera_y;
    final_z = camera_z;
    res.x = final_x;
    res.y = final_y;
    res.z = final_z;
    std::cout << "x  = " << camera_x << " y = " << camera_y
              << " z = " << camera_z << std::endl;
    std::cout << "service answer" << std::endl;
  }
  
  return true;
}


int main(int argc, char **argv) {
  //初始化ROS
  ros::init(argc, argv, "cood_tran");
  ros::NodeHandle nh;

  ros::ServiceServer location_server =
      nh.advertiseService("location_srv", location);
  ROS_INFO("ready srv server");

  //创建接收darknet结果和点云信息的接受者
  ros::Subscriber ros_coord_pixel_sub =
      nh.subscribe("/darknet_ros/bounding_boxes", 1, darknetCallback);
  /*/camera/depth_registered/points color_optical*/
  ros::Subscriber point_cloud_sub =
      nh.subscribe("/camera/depth_registered/points", 1, pointCouldCallback);

  //循环等待回调函数
  ros::spin();
  return 0;
}
