#include "ros/ros.h"
#include "cood_tran.h"


int main(int argc, char **argv) 
{

  ros::init(argc, argv, "cood_tran");
  ros::NodeHandle nh;
  
  coodTran(nh);

  ros::spin();
  return 0;
}
