cmake_minimum_required(VERSION 2.8.3)
project(ros_pub_sub)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    rospy
    roscpp
    nodelet
    geometry_msgs
    nav_msgs
    mrs_msgs
    mrs_lib
    message_generation
    std_msgs
    )

add_message_files(
  FILES
  DistTarget.msg
)

generate_messages(
  DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})
    
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_pub_sub
  CATKIN_DEPENDS rospy message_runtime roscpp mrs_lib mrs_msgs geometry_msgs nav_msgs
  DEPENDS Eigen
  )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

## Declare a C++ library
add_library(Ros_Pub src/ros_pub.cpp)
add_dependencies(Ros_Pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(Ros_Pub
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

add_library(Ros_Sub src/ros_sub.cpp)
add_dependencies(Ros_Sub ${catkin_EXPORTED_TARGETS})
target_link_libraries(Ros_Sub
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
