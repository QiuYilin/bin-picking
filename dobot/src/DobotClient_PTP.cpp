#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dobot/SetCmdTimeout.h"
#include "dobot/SetQueuedCmdClear.h"
#include "dobot/SetQueuedCmdStartExec.h"
#include "dobot/SetQueuedCmdForceStopExec.h"
#include "dobot/GetDeviceVersion.h"

#include "dobot/SetEndEffectorParams.h"
#include "dobot/SetPTPJointParams.h"
#include "dobot/SetPTPCoordinateParams.h"
#include "dobot/SetPTPJumpParams.h"
#include "dobot/SetPTPCommonParams.h"
#include "dobot/SetPTPCmd.h"
#include "dobot/SetEndEffectorSuctionCup.h"

#include <location_srv/Location.h>






int main(int argc, char **argv)
{
    //初始化客户端节点
    ros::init(argc, argv, "DobotClient");
    ros::NodeHandle n;

    // ros::Subscriber location_sub = nh.subscribe("location_msg", 1, dobotCallback);

    //创建Client实例，发出服务请求，等待服务端处理请求并响应
    ros::ServiceClient client;

    // SetCmdTimeout
    client = n.serviceClient<dobot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
    dobot::SetCmdTimeout srv1;
    srv1.request.timeout = 3000;
    if (client.call(srv1) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
        return -1;
    }

    // Clear the command queue
    client = n.serviceClient<dobot::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");
    dobot::SetQueuedCmdClear srv2;
    client.call(srv2);

    // Start running the command queue
    client = n.serviceClient<dobot::SetQueuedCmdStartExec>("/DobotServer/SetQueuedCmdStartExec");
    dobot::SetQueuedCmdStartExec srv3;
    client.call(srv3);

    // Get device version information
    client = n.serviceClient<dobot::GetDeviceVersion>("/DobotServer/GetDeviceVersion");
    dobot::GetDeviceVersion srv4;
    client.call(srv4);
    if (srv4.response.result == 0) {
        ROS_INFO("Device version:%d.%d.%d", srv4.response.majorVersion, srv4.response.minorVersion, srv4.response.revision);
    } else {
        ROS_ERROR("Failed to get device version information!");
    }

    // Set end effector parameters
    client = n.serviceClient<dobot::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams");
    dobot::SetEndEffectorParams srv5;
    srv5.request.xBias = 70;
    srv5.request.yBias = 0;
    srv5.request.zBias = 0;
    client.call(srv5);


    // Set PTP joint parameters
    do {
        client = n.serviceClient<dobot::SetPTPJointParams>("/DobotServer/SetPTPJointParams");
        dobot::SetPTPJointParams srv;

        for (int i = 0; i < 4; i++) {
            srv.request.velocity.push_back(100);
        }
        for (int i = 0; i < 4; i++) {
            srv.request.acceleration.push_back(100);
        }
        client.call(srv);
    } while (0);

    // Set PTP coordinate parameters
    do {
        client = n.serviceClient<dobot::SetPTPCoordinateParams>("/DobotServer/SetPTPCoordinateParams");
        dobot::SetPTPCoordinateParams srv;

        srv.request.xyzVelocity = 100;
        srv.request.xyzAcceleration = 100;
        srv.request.rVelocity = 100;
        srv.request.rAcceleration = 100;
        client.call(srv);
    } while (0);

    // Set PTP jump parameters
    do {
        client = n.serviceClient<dobot::SetPTPJumpParams>("/DobotServer/SetPTPJumpParams");
        dobot::SetPTPJumpParams srv;

        srv.request.jumpHeight = 20;
        srv.request.zLimit = 200;
        client.call(srv);
    } while (0);

    // Set PTP common parameters
    do {
        client = n.serviceClient<dobot::SetPTPCommonParams>("/DobotServer/SetPTPCommonParams");
        dobot::SetPTPCommonParams srv;

        srv.request.velocityRatio = 50;
        srv.request.accelerationRatio = 50;
        client.call(srv);
    } while (0);





    float fin_x,fin_y,fin_z;
    while (ros::ok()) {
        
        
        
        client = n.serviceClient<location_srv::Location>("location_srv");
        location_srv::Location srv6;
 
        
        if(client.call(srv6))
        {
            fin_x = srv6.response.x;
            fin_y = srv6.response.y;
            fin_z = srv6.response.z;
            ROS_INFO("fin_x,fin_y,fin_z: %f, %f,%f",fin_x,fin_y,fin_z);

        }
        else
        {
            ROS_INFO("Failed to call service location_srv");
            return 1;
        }

        do {
           //前往目标位置
           client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
           dobot::SetPTPCmd srv;
            srv.request.ptpMode = 1;
            srv.request.x = fin_x;
            srv.request.y = fin_y;
            srv.request.z = fin_z;
            srv.request.r = 0;
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }     
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (1);

        do {
           //开启吸盘
           client = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
           dobot::SetEndEffectorSuctionCup srv;
            srv.request.enableCtrl = 1;
            srv.request.suck = 1;

            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }     
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (1);

        do {
           //把东西放到一边
           client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
           dobot::SetPTPCmd srv;
            srv.request.ptpMode = 1;
            srv.request.x = 200;
            srv.request.y = 200;
            srv.request.z = 0;
            srv.request.r = 0;
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (1);

        
        do {
          //关闭吸盘       
           client = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
           dobot::SetEndEffectorSuctionCup srv;
            srv.request.suck = 0;
            srv.request.enableCtrl = 0;

            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }     
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (1);
        //回到起始位置

        do {
           client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
           dobot::SetPTPCmd srv;
            srv.request.ptpMode = 1;
            srv.request.x = 200;
            srv.request.y = 0;
            srv.request.z = 0;
            srv.request.r = 0;
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (1);
  
        ros::spinOnce();
    }

    return 0;
}

