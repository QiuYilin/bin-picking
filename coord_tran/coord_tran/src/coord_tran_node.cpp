#include "ros/ros.h"
#include "CoordinateTran.h"




int main(int argc, char **argv) 
{

  ros::init(argc, argv, "coord_tran");
  ros::NodeHandle nh("~");
#if   pictureCatch
        cv::namedWindow("view");
        cv::startWindowThread();
#endif 
  huskybot_arm::CoordinateTran coordTran(nh);
  ros::spin();
#if   pictureCatch
  cv::destroyWindow("view");
#endif 
  return 0;
}






