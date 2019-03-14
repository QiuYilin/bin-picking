cmake_minimum_required(VERSION 2.8.3)
project(cood_tran)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS darknet_ros_msgs roscpp sensor_msgs image_transport std_msgs pcl_ros location_srv cv_bridge)

find_package(OpenCV REQUIRED)


catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARIES}
)




add_executable(cood_tran
  src/cood_tran.cpp
)
add_dependencies(cood_tran ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cood_tran
  ${catkin_LIBRARIES}
)

add_executable(cood_tran_depth src/cood_tran_depth.cpp)
 target_link_libraries(cood_tran_depth
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
add_dependencies(cood_tran_depth ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cood_tran_depth
  ${catkin_LIBRARIES}
)


SET(CMAKE_BUILD_TYPE "Debug")  
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
