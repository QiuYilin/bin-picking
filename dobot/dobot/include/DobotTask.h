#ifndef DOBOTTASK_H
#define DOBOTTASK_H
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
#include "dobot/GetPose.h"

#include <coord_tran_msgs/location.h>
#include <arm_msgs/arm_car_interact.h>

#include "dobot/SetIODO.h"
#include "dobot/SetIOMultiplexing.h"



#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>


float start_pose_x = 190, start_pose_y = 160 , start_pose_z = 100;
float tool_length = 120;
#endif