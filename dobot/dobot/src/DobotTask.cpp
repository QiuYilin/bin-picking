#include "DobotTask.h"



ros::ServiceClient client;
bool get_target;
int x,y,z,r;

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
    srv5.request.xBias = 59.7;//末端执行器偏移距离**
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

        srv.request.jumpHeight = 5;//提升距离，太大会超出工作范围**
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


        srv.request.x = start_pose_x;
        srv.request.y = start_pose_y;
        srv.request.z = start_pose_z;
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
                 std::cout<<"读取当前机械臂位姿"<<std::endl;       
                 client = n.serviceClient<dobot::GetPose>("/DobotServer/GetPose");
                 dobot::GetPose srv;
                 
                 client.call(srv);
                 if (srv.response.result==0) {               
                 std::cout << "x" << srv.response.x << "y" << srv.response.y << "z" << srv.response.z <<std::endl;
                      break;
                    }     
                 ros::spinOnce();
                 if (ros::ok() == false) {
                        break;
                    }
                } while (1);




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

    
}

void dobotTask(ros::NodeHandle &n, float x, float y, float z)
{

                    do {
                    std::cout<<"前往目标位置"<<std::endl;
                    client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
                    dobot::SetPTPCmd srv;
                    srv.request.ptpMode = 0;//????

                    srv.request.x = x;
                    srv.request.y = y;
                    srv.request.z = z;
                    srv.request.r = r;
		     std::cout << "x2 "<<x<< "y2 "<<y<< "z2 "<<z<< std::endl;
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
                    srv.request.suck = 1;
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
                        srv.request.ptpMode = 0;
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

                    
                    do {
                    std::cout<<" 关闭吸盘   "<<std::endl;       
                    client = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
                    dobot::SetEndEffectorSuctionCup srv;
                        srv.request.suck = 0;
                        srv.request.enableCtrl = 0;
                        srv.request.isQueued = 1;//要加到队列**
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
                        srv.request.ptpMode = 0;


                        srv.request.x = start_pose_x;
                        srv.request.y = start_pose_y;
                        srv.request.z = start_pose_z;

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

void dobotTask_soft(ros::NodeHandle &n, float x, float y, float z,float r)
{
                    do {
                    std::cout<<"开启吸盘 "<<std::endl;
                    client = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
                    dobot::SetEndEffectorSuctionCup srv;
                    srv.request.enableCtrl = 1;
                    srv.request.suck = 1;
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

                    
                    do {
                    std::cout<<" 等待   "<<std::endl;       
                    client = n.serviceClient<dobot::SetWAITCmd>("/DobotServer/SetWAITCmd");
                    dobot::SetWAITCmd srv;
                        srv.request.timeout = 3000;
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
                   
                    client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
		   
                    dobot::SetPTPCmd srv;
                    srv.request.ptpMode = 0;//????

                    srv.request.x = x;
                    srv.request.y = y;
                    srv.request.z = z;
                    srv.request.r = r;
		    std::cout << "x2 "<<x<< "y2 "<<y<< "z2 "<<z<<"r2 "<<r <<std::endl;
		    // ros::service::waitForService("/DobotServer/SetPTPCmd");
                    client.call(srv);
                    if (srv.response.result == 0) {
		       std::cout<<"前往目标位置"<<std::endl;
                        break;
                    }     
                    ros::spinOnce();
                    if (ros::ok() == false) {
                        break;
                    }
                    } while (1);
		    
// 		    do {
//                     std::cout<<" 等待   "<<std::endl;       
//                     client = n.serviceClient<dobot::SetWAITCmd>("/DobotServer/SetWAITCmd");
//                     dobot::SetWAITCmd srv;
//                         srv.request.timeout = 5000;
//                         client.call(srv);
//                      if (srv.response.result == 0) {
//                         break;
//                     }     
//                     ros::spinOnce();
//                     if (ros::ok() == false) {
//                         break;
//                     }
//                     } while (1);

                    do{
                    client = n.serviceClient<dobot::SetIODO>("/DobotServer/SetIODO");
                    dobot::SetIODO srv1;
                    srv1.request.address = 17;
                    srv1.request.level = 1;
                    srv1.request.isQueued = 1;
                    client.call(srv1);
                    if (srv1.response.result == 0) {
                    std::cout<<" 软体手启动   "<<std::endl;
                    break;
                    }
                    if (ros::ok() == false) 
                    break;
                    }while(1); 


                      do {
                    std::cout<<" 等待   "<<std::endl;       
                    client = n.serviceClient<dobot::SetWAITCmd>("/DobotServer/SetWAITCmd");
                    dobot::SetWAITCmd srv;
                        srv.request.timeout = 5000;
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
                        srv.request.ptpMode = 0;
                        srv.request.x = 0;
                        srv.request.y = 200;
                        srv.request.z = 90;
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

            // //机械臂是否到了存放位置
            //      do {
            //      std::cout<<"读取当前机械臂位姿"<<std::endl;       
            //      client = n.serviceClient<dobot::GetPose>("/DobotServer/GetPose");
            //      dobot::GetPose srv;
                 
            //      client.call(srv);
            //      if (srv.response.result==0&&srv.response.x == 0&&srv.response.y==200&&srv.response.z==70) {               
            //           break;
            //         }     
            //      ros::spinOnce();
            //      if (ros::ok() == false) {
            //             break;
            //         }
            //     } while (1);

                    
                    do{
                    client = n.serviceClient<dobot::SetIODO>("/DobotServer/SetIODO");
                    dobot::SetIODO srv1;
                    srv1.request.address = 17;
                    srv1.request.level = 0;
                    srv1.request.isQueued = 1;
                    client.call(srv1);
                    if (srv1.response.result == 0) {
                    std::cout<<" 软体手关闭   "<<std::endl;
                    break;
                    }
                    if (ros::ok() == false) 
                    break;
                    }while(1); 

                      do {
                    std::cout<<" 等待   "<<std::endl;       
                    client = n.serviceClient<dobot:: SetWAITCmd>("/DobotServer/SetWAITCmd");
                    dobot:: SetWAITCmd srv;
                        srv.request.timeout = 7000;
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

                        srv.request.x = start_pose_x;
                        srv.request.y = start_pose_y;
                        srv.request.z = start_pose_z;
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
    ros::service::waitForService("/coord_tran/location_srv");
    client = n2.serviceClient<coord_tran_msgs::location>("/coord_tran/location_srv");
    coord_tran_msgs::location srv6;
    
    if(client.call(srv6))
    {
        get_target = srv6.response.get_target;
	
	//Eigen::Vector4f PC(srv6.response.x,srv6.response.y,srv6.response.z,1);
	std::cout<< "camera_x " << srv6.response.x <<" camera_y "<<srv6.response.y<<" camera_z "<<srv6.response.z<<"r"<<r<<std::endl;
//         Eigen::Matrix4f C2B3;
//         C2B3 << 1,0,0,0.07052,
// 	0,1,0,0.00274 ,
// 	0,0,1,0.02961,
// 	0,0,0,1;
// 	Eigen::Matrix4f C2B2;
//         C2B2 << 0,1,0,0,
// 	-1,0,0,0 ,
// 	0,0,1,0,
// 	0,0,0,1;
// 	Eigen::Matrix4f C2B1;
//         C2B1 << 1,0,0,0,
// 	0,-1,0,0 ,
// 	0,0,-1,0 ,
// 	0,0,0,1;
// 	Eigen::Matrix4f B2A;
// 	B2A << 1,0,0, start_pose_x/1000,
// 	0,1,0,start_pose_y/1000,
// 	0,1,0,start_pose_z/1000 ,
// 	0,0,0,1;
// 	Eigen::Vector4f PA= B2A*(C2B3*C2B2*C2B1)*PC;
// 	std::cout<< "PA(0) " << PA(0) <<"PA(1)"<<PA(1)<<"PA(2)"<<PA(2)<<"r"<<r<<std::endl;
//         x = 1000*PA(0);
//         y =1000*PA(1);
//         z =1000*PA(2)-tool_length;
	x=-1000*srv6.response.y+70.52+start_pose_x+0.01;
	y=-1000*srv6.response.x+2.74+start_pose_y+0.01;
	//z=-1000*srv6.response.z+29.61+start_pose_z+tool_length;
	z=-1000*srv6.response.z+10.5+start_pose_z-50-9;
	r=srv6.response.r;//-30<r<150
	if (r<-30)
	  r=180+r;
	
// 	if (r>180)
// 	r=r-180;

    }
    else
    std::cout<<"client is failed "<<std::endl;

    float d = sqrt(x*x+y*y);
    if(get_target==1)
    {
        
        std::cout<<"机械臂运行"<<std::endl;

        if(d>200&&d<315)
        {
            dobotTask_soft(n2,x,y,z,r); 
            //等待2秒,确保机械臂已经开始运动
            do {
                 std::cout<<" 等待   "<<std::endl;       
                 client = n2.serviceClient<dobot:: SetWAITCmd>("/DobotServer/SetWAITCmd");
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
            //机械臂是否已经回到了初始位置
            do {
                 //std::cout<<"读取当前机械臂位姿"<<std::endl;       
                 client = n2.serviceClient<dobot::GetPose>("/DobotServer/GetPose");
                 dobot::GetPose srv;
                 
                 client.call(srv);
                 if (srv.response.result==0&&srv.response.x == start_pose_x&&srv.response.y==start_pose_y&&srv.response.z==start_pose_z) {               
                      break;
                    }     
                 ros::spinOnce();
                 if (ros::ok() == false) {
                        break;
                    }
                } while (1);
                res.result = 0;

        }
        else
        {
            std::cout<<"仍然不在机械臂工作空间"<<std::endl;

            if (d < 0.2)
            std::cout<<"目前距离底座中心： "<< (0.2- d)<< "m" <<std::endl;
            if (d > 0.315)
            std::cout<<"目前距离底座中心： "<< (d - 0.315)<<"m"<<std::endl;

            if (d < 0.2||d > 0.315)
            std::cout<<"目前距离底座中心： "<< d<< "m" <<std::endl;

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
 


int main(int argc, char **argv){
    ros::init(argc, argv, "DobotTask");
    ros::NodeHandle n;
    dobotInit(n);

    //ros::param::get("grasp_on", grasp_on);

    ros::ServiceServer arm_car_server = n.advertiseService("arm_car_srv",arm_car_interact);
    ros::spin();
    return 0;
}

