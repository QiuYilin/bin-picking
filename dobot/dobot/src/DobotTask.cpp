#include "DobotTask.h"



ros::ServiceClient client;
bool get_target;
float x,y,z;

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
}

void dobotTask(ros::NodeHandle &n)
{

                    do {
                    std::cout<<"前往目标位置"<<std::endl;
                    client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
                    dobot::SetPTPCmd srv;
                    srv.request.ptpMode = 1;
                    srv.request.x = 300;
                    srv.request.y = 0;
                    srv.request.z = 10;
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
                    std::cout<<"开启吸盘 "<<std::endl;
                    client = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
                    dobot::SetEndEffectorSuctionCup srv;
                    srv.request.enableCtrl = 1;
                    srv.request.suck = 0;
                    client.call(srv);
                     if (srv.response.result == 0) {
                        break;
                    }     
                    ros::spinOnce();
                    if (ros::ok() == false) {
                        break;
                    }
                    }while (1);

                      do {
                    std::cout<<" 等待   "<<std::endl;       
                    client = n.serviceClient<dobot::SetWAITCmd>("/DobotServer/SetWAITCmd");
                    dobot::SetWAITCmd srv;
                        srv.request.timeout = 2000;
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
                    std::cout<<" 把东西放到一边  "<<std::endl;
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
                    std::cout<<" 关闭吸盘   "<<std::endl;       
                    client = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
                    dobot::SetEndEffectorSuctionCup srv;
                        srv.request.suck = 1;
                        srv.request.enableCtrl = 1;
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
                    std::cout<<" 等待   "<<std::endl;       
                    client = n.serviceClient<dobot:: SetWAITCmd>("/DobotServer/SetWAITCmd");
                    dobot:: SetWAITCmd srv;
                        srv.request.timeout = 2000;
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
                      std::cout<<"  回到起始位置   "<<std::endl;     
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
}


bool arm_car_interact(arm_msgs::arm_car_interact::Request &req,arm_msgs::arm_car_interact::Response &res)
{
    ros::NodeHandle n2;
    //读取目标是否识别到以及目标相对于机械臂的坐标
    client = n2.serviceClient<cood_tran_msgs::location>("/coord_tran/location_srv");
    cood_tran_msgs::location srv6;
    if(client.call(srv6))
    {
        get_target = srv6.response.get_target;
        x = srv6.response.x;
        y = srv6.response.y;
        z = srv6.response.z;
    }
    if(get_target==1)
    {
        float r = sqrt(x*x+y*y);
        std::cout<<"机械臂运行"<<std::endl;
        if(r>200&&r<315)
        {
            dobotTask(n2); 
            res.result = 0;
        }
        else
        {
            std::cout<<"仍然不在机械臂工作空间"<<std::endl;
            res.result = 1;
        }
    }
    else
    {
        std::cout<<"没有识别到物体"<<std::endl;
        res.result = 2;
    }
    return true;
}
 


int main(int argc, char **argv)
{
    ros::init(argc, argv, "DobotTask");
    ros::NodeHandle n;
    dobotInit(n);

    //ros::param::get("grasp_on", grasp_on);

    ros::ServiceServer arm_car_server = n.advertiseService("arm_car_srv",arm_car_interact);
    return 0;
}

