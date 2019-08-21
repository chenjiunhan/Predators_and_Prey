cmake_minimum_required(VERSION 2.8.3)
project(robobo_gazebo)

find_package(catkin REQUIRED COMPONENTS  
    roscpp
    rospy
    std_msgs
    robobo_msgs
    gazebo
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_library(robobo_set_wheel_joint_velocity SHARED plugin/robobo_set_wheel_joint_velocity.cc)
target_link_libraries(robobo_set_wheel_joint_velocity ${GAZEBO_LIBRARIES})
