cmake_minimum_required(VERSION 3.1.3)

project(brick_search)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        image_transport
        cv_bridge
        sensor_msgs
        geometry_msgs
        nav_msgs
        tf2_ros
        std_srvs
        actionlib
        move_base_msgs
        )

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}-program src/main.cpp)

add_library(${PROJECT_NAME}-BrickSearch src/BrickSearch.cpp)

target_include_directories(${PROJECT_NAME}-program SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

target_include_directories(${PROJECT_NAME}-program PRIVATE include)

target_link_libraries(${PROJECT_NAME}-program ${OpenCV_LIBS} ${catkin_LIBRARIES})

