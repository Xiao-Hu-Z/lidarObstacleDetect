cmake_minimum_required(VERSION 2.8.3)
project(lidar_detection_track)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
# # Debug
# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


# find_package(CUDA REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  jsk_rviz_plugins
  pcl_ros
  roscpp
  rospy
  std_msgs
  roslint
)


catkin_package(
  INCLUDE_DIRS 
  LIBRARIES lidar_detect
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${CUDA_INCLUDE_DIRS}
)

# 必须加上
#file(GLOB TENSORRT_LIBS lib/detection/point_pillars/TensorRT-7.2.3.4/lib/*.so)

# 添加子目录并构建

add_subdirectory(lib/pre_process/roi_clip)
add_subdirectory(lib/pre_process/voxel_grid_filter)
add_subdirectory(lib/cluster/euclideanCluster)
add_subdirectory(lib/bounding_box)
add_subdirectory(lib/ground_detector/patchwork)
# add_subdirectory(lib/detection/point_pillars)
add_subdirectory(lib/visualization)
# add_subdirectory(lib/cluster/cluster_c)


# 生成可执行文件,ros启动就不要改默认路径
 set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../) # 设置可执行目标文件的输出目录

# cuda_add_executable(${PROJECT_NAME}_node src/lidar_detection_track_node.cpp src/lidar_detection_track.cpp )
add_executable(${PROJECT_NAME}_node src/lidar_detection_track_node.cpp src/lidar_detection_track.cpp  )
# set_target_properties(${PROJECT_NAME}_node PROPERTIES CUDA_ARCHITECTURES "35;50;72;86")

target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  # ${CUDA_LIBRARIES}


  roi_clip
  voxel_grid_filter
  euclideanCluster
  bounding_box
  patchwork
  visualization
  # point_pillars
  ${TENSORRT_LIBS}
 )

 # 在PRIVATE后面的库仅被link到你的target中，并且终结掉，第三方不能感知你掉了啥库
 # 在public后面的库会被link到你的target中，并且里面的符号也会被导入出，提供给给三方使用
 # INTERFACE 在interface后面引入的库不会被链接到target中，只会导出符号

# cuda_add_executable(${PROJECT_NAME}_node src/lidar_euclidean_cluster_detect.cpp)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES CUDA_ARCHITECTURES "35;50;72;86")
# target_link_libraries(${PROJECT_NAME}_node  PRIVATE ${CUDA_LIBRARIES})
#  target_link_libraries(${PROJECT_NAME}_node  PRIVATE ${DETECT_LIB_NAME})
#  target_link_libraries(${PROJECT_NAME}_node  PRIVATE ${TENSORRT_LIBS})
#  target_link_libraries(${PROJECT_NAME}_node  PRIVATE ${catkin_LIBRARIES})
#  target_link_libraries(${PROJECT_NAME}_node  PRIVATE cluster)
#  target_link_libraries(${PROJECT_NAME}_node  PRIVATE preprocess)
#  target_link_libraries(${PROJECT_NAME}_node  PRIVATE detect_point_pillars)
#  target_link_libraries(${PROJECT_NAME}_node  PRIVATE visualization)

