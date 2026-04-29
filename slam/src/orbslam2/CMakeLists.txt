cmake_minimum_required(VERSION 3.5)
project(orbslam2)

# Cmake modules folder
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

find_package(Boost COMPONENTS system) 
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

# ORB_SLAM2 is an external (non-ROS) library and may not be installed.
# Allow the workspace to build without it; when present, build the wrapper node.
option(ORB_SLAM2_ENABLE "Build ORB-SLAM2 wrapper node" ON)
if(ORB_SLAM2_ENABLE)
  find_package(ORB_SLAM2 QUIET)
  if(NOT ORB_SLAM2_FOUND)
    message(WARNING
      "ORB_SLAM2 not found; skipping orbslam2 node build. "
      "Set ORB_SLAM2_ROOT_DIR (or install ORB_SLAM2) and rebuild to enable.")
    ament_package()
    return()
  endif()
endif()

include_directories(
  ${ORB_SLAM2_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)

# Monocular
add_executable(
    mono 
    src/monocular/mono.cpp
    src/monocular/monocular-slam-node.cpp
)

target_link_libraries(mono
  ${ORB_SLAM2_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
)

ament_target_dependencies(mono rclcpp sensor_msgs visualization_msgs cv_bridge message_filters)

install(TARGETS mono DESTINATION lib/${PROJECT_NAME})
ament_package()
