cmake_minimum_required(VERSION 2.8.3)
project(coord_tran)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS darknet_ros_msgs roscpp sensor_msgs image_transport std_msgs pcl_ros coord_tran_msgs cv_bridge OpenCV)

find_package(OpenCV REQUIRED)
# find_package(Super4PCS REQUIRED)
find_package( Eigen3 REQUIRED )

catkin_package()



include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_INCLUDE_DIR}
  #${Super4PCS_INCLUDE_DIR}
)




add_executable(coord_tran_node
  src/coord_tran_node.cpp src/CoordinateTran.cpp src/threeDdeal.cpp 
)

add_dependencies(coord_tran_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(coord_tran_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


add_executable(threeDtest
  src/threeDtest.cpp 
)

add_dependencies(threeDtest  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(threeDtest
  ${catkin_LIBRARIES}
)

SET(CMAKE_BUILD_TYPE "Debug")  
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
