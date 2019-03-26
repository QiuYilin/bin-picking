#include "ros/ros.h"
#include "cood_tran.h"

extern std::string target_obj;

int main(int argc, char **argv) 
{
  //初始化ROS
  ros::init(argc, argv, "cood_tran");
  ros::NodeHandle nh;
  target_obj = "None";
  ros::param::get("target", target_obj);
  std::cout << "param  = " << target_obj << std::endl;
  //创建接收darknet结果和点云信息的接受者
  ros::Subscriber ros_coord_pixel_sub =
  nh.subscribe("/darknet_ros/bounding_boxes", 1, darknetCallback);
  /*/camera/depth_registered/points color_optical*/
  ros::Subscriber point_cloud_sub =
  nh.subscribe("camera/depth_registered/points", 1, pointCouldCallback);///camera/depth_registered/points
  ros::ServiceServer location_server =
  nh.advertiseService("location_srv", location);
  ROS_INFO("ready srv server");
  //循环等待回调函数
  ros::spin();
  return 0;
}
