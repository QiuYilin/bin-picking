#include "ros/ros.h"
#include <location_srv/Location.h>
#include <cstdlib>

int main(int argc,char **argv)
{
    ros::init(argc, argv, "testclient_location");

    ros::NodeHandle nh;

    ros::ServiceClient testclient_location = nh.serviceClient<location_srv::Location>("location_srv");

    location_srv::Location srv;

    if(testclient_location.call(srv))
        {
            float fin_x,fin_y,fin_z;
            fin_x = srv.response.x;
            fin_y = srv.response.y;
            fin_z = srv.response.z;
            ROS_INFO("fin_x,fin_y,fin_z: %f, %f,%f",fin_x,fin_y,fin_z);

        }
    else
        {
            ROS_INFO("Failed to call service location_srv");
            return 1;
        }    
    return 0;
}




        
