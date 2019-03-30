#include "ros/ros.h"
#include "cood_tran.h"


extern std::string target_obj;



int main(int argc, char **argv) 
{

  ros::init(argc, argv, "cood_tran");
  ros::NodeHandle nh;
  
  std::cout << "coodTran" << std::endl;
  target_obj = "None";
  ros::param::get("target", target_obj);

  ros::Subscriber ros_coord_pixel_sub =
  nh.subscribe("/darknet_ros/bounding_boxes", 1, darknetCallback);

  ros::Subscriber point_cloud_sub =
  nh.subscribe("camera/depth_registered/points", 1, pointCouldCallback);///camera/depth_registered/points  /camera/depth_registered/points<->color_optical
  
  ros::ServiceServer location_server =
  nh.advertiseService("location_srv", location);


  ros::spin();
  return 0;
}






