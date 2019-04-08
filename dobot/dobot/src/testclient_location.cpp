#include "ros/ros.h"
#include <cood_tran_msgs/location.h>
#include <cstdlib>
#define armLogicDebug 1
int main(int argc,char **argv)
{
    ros::init(argc, argv, "testclient_location");

    ros::NodeHandle nh;

    ros::ServiceClient testclient_location = nh.serviceClient<cood_tran_msgs::location>("location_srv");

    cood_tran_msgs::location srv;
    float fin_x,fin_y,fin_z;
while(1)
{
    
    if(testclient_location.call(srv))
        { 
            ROS_INFO("get_target: %d",srv.response.get_target);
            if(srv.response.get_target == 1 )
           {
                fin_x = srv.response.x;
                fin_y = srv.response.y;
                fin_z = srv.response.z;
                ROS_INFO("fin_x,fin_y,fin_z: %f, %f,%f",fin_x,fin_y,fin_z);
            }
            else
            {
                ROS_INFO("No target");
            }
        }
    else
        {
            ROS_INFO("Failed to call service location!");
        }    
}
    return 0;
}




        
