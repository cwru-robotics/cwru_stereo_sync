cmake_minimum_required(VERSION 2.8.3)
project(cwru_stereo_sync)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  image_transport
  roscpp
  roslint
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(stereo_sync_node src/cameras_timesync.cpp)

target_link_libraries(stereo_sync_node
    ${catkin_LIBRARIES}
)


roslint_cpp()
roslint_add_test()