cmake_minimum_required(VERSION 2.8.3)
project(cood_tran_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_service_files(FILES
  location.srv
  car_interact.srv
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)


include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)


