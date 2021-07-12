cmake_minimum_required(VERSION 3.5)
project(vicon_stream)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  tf2
)
find_package(PCL REQUIRED)
find_package(libmotioncapture REQUIRED)

catkin_package(
 CATKIN_DEPENDS roscpp geometry_msgs std_msgs tf2
 DEPENDS PCL libmotioncapture
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${libmotioncapture_INCLUDE_DIRS}
)

# Enable Vicon
add_definitions(-DENABLE_VICON)

add_executable(vicon_stream
  src/vicon_stream.cpp
)
target_link_libraries(vicon_stream
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${libmotioncapture_LIBRARIES}
)
