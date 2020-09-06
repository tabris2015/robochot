cmake_minimum_required(VERSION 3.0.2)
project(robochot)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robochot
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robochot_base src/robochot_base.cpp)
target_link_libraries(robochot_base ${catkin_LIBRARIES})