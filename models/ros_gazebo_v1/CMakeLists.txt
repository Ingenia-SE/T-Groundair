cmake_minimum_required(VERSION 2.8.3)
project(ros_gazebo_v1)

# Check for c++11 / c++0x support
add_definitions(-std=c++11)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
  cv_bridge
  image_transport
  rospy
  std_msgs
  genmsg
  message_generation
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

find_package(OpenCV 3.2 REQUIRED)

add_message_files(
  FILES
  balls.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
    cv_bridge
    image_transport
    rospy
    std_msgs
    message_generation
)

add_library(central_station_plugin SHARED src/central_station.cc)
target_link_libraries(central_station_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(drone_plugin SHARED src/drone.cc)
target_link_libraries(drone_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(interface src/interface.cpp)
target_link_libraries(interface ${catkin_LIBRARIES})

