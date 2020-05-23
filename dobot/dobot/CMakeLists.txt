cmake_minimum_required(VERSION 2.8.3)
project(dobot)

## Add support for C++11, supported in ROS Kinetic and newer
 add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
  geometry_msgs
  coord_tran_msgs
)

find_package( Eigen3 REQUIRED )

include_directories(
  
# include
  
  ${OpenCV_LIBRARIES}
  ${EIGEN3_INCLUDE_DIR}
  #${Super4PCS_INCLUDE_DIR}
)
add_message_files(
  FILES
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetCmdTimeout.srv

  GetDeviceSN.srv
  SetDeviceName.srv
  GetDeviceName.srv
  GetDeviceVersion.srv

  GetPose.srv

  GetAlarmsState.srv
  ClearAllAlarmsState.srv

  SetHOMEParams.srv
  GetHOMEParams.srv
  SetHOMECmd.srv

  SetEndEffectorParams.srv
  GetEndEffectorParams.srv
  SetEndEffectorLaser.srv
  GetEndEffectorLaser.srv
  SetEndEffectorSuctionCup.srv
  GetEndEffectorSuctionCup.srv
  SetEndEffectorGripper.srv
  GetEndEffectorGripper.srv

  SetJOGJointParams.srv
  GetJOGJointParams.srv
  SetJOGCoordinateParams.srv
  GetJOGCoordinateParams.srv
  SetJOGCommonParams.srv
  GetJOGCommonParams.srv
  SetJOGCmd.srv

  SetPTPJointParams.srv
  GetPTPJointParams.srv
  SetPTPCoordinateParams.srv
  GetPTPCoordinateParams.srv
  SetPTPJumpParams.srv
  GetPTPJumpParams.srv
  SetPTPCommonParams.srv
  GetPTPCommonParams.srv
  SetPTPCmd.srv

  SetCPParams.srv
  GetCPParams.srv
  SetCPCmd.srv

  SetARCParams.srv
  GetARCParams.srv
  SetARCCmd.srv

  SetWAITCmd.srv

  SetTRIGCmd.srv

  SetIOMultiplexing.srv
  GetIOMultiplexing.srv
  SetIODO.srv
  GetIODO.srv
  SetIOPWM.srv
  GetIOPWM.srv
  GetIODI.srv
  GetIOADC.srv
  SetEMotor.srv
  GetInfraredSensor.srv
  SetInfraredSensor.srv
  
  SetColorSensor.srv
  GetColorSensor.srv

  SetQueuedCmdStartExec.srv
  SetQueuedCmdStopExec.srv
  SetQueuedCmdForceStopExec.srv
  SetQueuedCmdClear.srv

  
)


generate_messages(
  DEPENDENCIES
  std_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dobot
CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs coord_tran_msgs
#  DEPENDS system_lib
)



if (CMAKE_SIZEOF_VOID_P EQUAL 4)
    include_directories(
      ${catkin_INCLUDE_DIRS}
      src/DobotDll_x86
    )

    link_directories(
      src/DobotDll_x86
    )
else()
    include_directories(
      ${catkin_INCLUDE_DIRS}
      src/DobotDll_x64

    )
    


    link_directories(
      src/DobotDll_x64
    )
endif()



# add_executable(testclient_location src/testclient_location.cpp)
# add_dependencies(testclient_location ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(testclient_location
#   ${catkin_LIBRARIES}
# )




add_executable(DobotServer src/DobotServer.cpp)
target_link_libraries(DobotServer ${catkin_LIBRARIES} DobotDll)
add_dependencies(DobotServer dobot_gencpp)

add_executable(DobotClient_PTP src/DobotClient_PTP.cpp)
target_link_libraries(DobotClient_PTP ${catkin_LIBRARIES})
add_dependencies(DobotClient_PTP dobot_gencpp)

add_executable(DobotClient_JOG src/DobotClient_JOG.cpp)
target_link_libraries(DobotClient_JOG ${catkin_LIBRARIES})
add_dependencies(DobotClient_JOG dobot_gencpp)

add_executable(DobotTask src/DobotTask.cpp)
target_link_libraries(DobotTask ${catkin_LIBRARIES})
add_dependencies(DobotTask dobot_gencpp)

add_executable(iotest src/iotest.cpp)
target_link_libraries(iotest ${catkin_LIBRARIES})
add_dependencies(iotest dobot_gencpp)

add_executable(iotest16 src/iotest16.cpp)
target_link_libraries(iotest16 ${catkin_LIBRARIES})
add_dependencies(iotest16 dobot_gencpp)

add_executable(iotest_close src/iotest_close.cpp)
target_link_libraries(iotest_close ${catkin_LIBRARIES})
add_dependencies(iotest_close dobot_gencpp)

add_executable(r_test src/r_test.cpp)
target_link_libraries(r_test ${catkin_LIBRARIES})
add_dependencies(r_test dobot_gencpp)

include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
  ${catkin_INCLUDE_DIRS}

  #${Super4PCS_INCLUDE_DIR}
)
