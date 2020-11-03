cmake_minimum_required(VERSION 2.8.3)
project(camlidar_logger)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  message_filters
  image_transport
)

catkin_package(
	DEPENDS Eigen
  CATKIN_DEPENDS roscpp
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${Eigen_INCLUDE_DIRS}
)

set(SRC ${SRC} 
src/main.cpp)

set(DCMAKE_build_type Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -O2 -m64 -Ofast -flto -msse3 -march=native -mfpmath=sse")

add_executable (camlidar_logger_node ${SRC} )
target_link_libraries (camlidar_logger_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})