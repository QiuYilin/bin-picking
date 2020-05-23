#include "ros/ros.h"
#include <math.h>
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
#include "dobot/SetWAITCmd.h"
#include "dobot/SetIODO.h"
#include "dobot/SetIOMultiplexing.h"
#include "dobot/GetIOMultiplexing.h"
#include "dobot/GetPose.h"
#define io 0 //0 16 1 17

ros::ServiceClient client;
void dobotInit(ros::NodeHandle &n)
{

    std::cout<<"初始化 "<<std::endl;
    // SetCmdTimeout
    client = n.serviceClient<dobot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
    dobot::SetCmdTimeout srv1;
    srv1.request.timeout = 3000;
    if (client.call(srv1) == false) {
        ROS_ERROR("Wait fot dobotserver to be on.");
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
                        
    do {
        std::cout<<"前往初始位置"<<std::endl;
        client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
        dobot::SetPTPCmd srv;
        srv.request.ptpMode = 1;

        srv.request.x = 200;
        srv.request.y = 0;
        srv.request.z = 70;

        srv.request.x = 200;
        srv.request.y = 0;
        srv.request.z = 70;
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
}

int main(int argc, char **argv){

    ros::init(argc, argv, "IOtest");
    ros::NodeHandle n;
    dobotInit(n);
    do{
    //IO复用
    client = n.serviceClient<dobot::SetIOMultiplexing>("/DobotServer/SetIOMultiplexing");
    dobot::SetIOMultiplexing srv;
    srv.request.address = 17;
    srv.request.multiplex = 1;
    client.call(srv);
     if (srv.response.result == 0) {
     std::cout<<" 复用   "<<std::endl;
     break;
    }
     if (ros::ok() == false) 
     break;
    }while(1);

    while(ros::ok()){
#if io
    do{
          
    client = n.serviceClient<dobot::SetIODO>("/DobotServer/SetIODO");
    dobot::SetIODO srv1;
    srv1.request.address = 17;
    srv1.request.level = 0;
    srv1.request.isQueued = 1;
    client.call(srv1);
    if (srv1.response.result == 0) {
     std::cout<<" 灯亮   "<<std::endl;
     break;
    }
     if (ros::ok() == false) 
     break;
     
    }while(1); 
#endif   
#if !io
                    do {
                    std::cout<<"负压关闭 "<<std::endl;
                    client = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
                    dobot::SetEndEffectorSuctionCup srv;
                    srv.request.enableCtrl = 0;
                    srv.request.suck = 0;
                    srv.request.isQueued = 1;//要加到队列**
                    client.call(srv);
                     if (srv.response.result == 0) {
                        break;
                    }     
                    ros::spinOnce();
                    if (ros::ok() == false) {
                        break;
                    }
                    }while (1);
#endif
    }
    ros::spin();
    return 0;
}