cmake_minimum_required(VERSION 3.0.2)
project(eskf_ndt)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  velodyne_msgs  
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)



set(TBB_ROOT_DIR "./Thirdparty/oneTBB-2020_U3")
set(TBB_BUILD_DIR "tbb_build_dir=${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
# ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}表示构建生成的库文件的输出目录,在build目录下
set(TBB_BUILD_PREFIX "tbb_build_prefix=tbb")

# 用于给生成的构建文件添加前缀
include(${TBB_ROOT_DIR}/cmake/TBBBuild.cmake)

# 引入TBB的构建配置文件，这个配置文件包含了 TBB 的构建规则和变量设置
tbb_build(
  TBB_ROOT ${TBB_ROOT_DIR}
  compiler=gcc-9
  stdver=c++17
  ${TBB_BUILD_DIR}
  ${TBB_BUILD_PREFIX}
  CONFIG_DIR
  TBB_DIR
  )

find_package(TBB REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories("./Thirdparty/sophus")
include_directories("./Thirdparty/oneTBB-2020_U3/include")

link_directories(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/tbb_release)

add_library(matching SHARED
    include/matching/eskf_ndt.cc
    include/matching/ndt.cc
)


target_link_libraries(matching TBB::tbb)

add_executable(eskf_ndt_matching src/eskf_ndt_matching.cpp)
target_link_libraries(eskf_ndt_matching 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES} 
  # ${Sophus_LIBRARIES} fmt 
  TBB::tbb
  matching
)

add_executable(map_manager src/map_manager.cpp)
target_link_libraries(map_manager 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES} 
)

add_executable(init_pose_publisher src/init_pose_publisher.cpp)
target_link_libraries(init_pose_publisher 
  ${catkin_LIBRARIES}
)

