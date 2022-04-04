cmake_minimum_required(VERSION 2.8.3)
project(euclideanCluster)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  jsk_rviz_plugins
)

find_package(OpenCV REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



aux_source_directory(. DIR_SRCS)

#SHARED 动态库
#STATIC 静态库
add_library(euclideanCluster STATIC ${DIR_SRCS})

target_link_libraries(euclideanCluster
  ${OpenCV_LIBRARIES}
  # ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
 )

