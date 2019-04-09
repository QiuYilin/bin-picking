#include "ros/ros.h"
#include "CoordinateTran.h"




int main(int argc, char **argv) 
{

  ros::init(argc, argv, "coord_tran");
  ros::NodeHandle nh("~");
  huskybot_arm::CoordinateTran coordTran(nh);
  ros::spin();
  return 0;
}






