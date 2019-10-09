cmake_minimum_required(VERSION 3.1.3)

project(maze_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nav_msgs
        geometry_msgs
        visualization_msgs
        message_generation
        )

add_service_files(FILES
        PlanPath.srv
        )

 generate_messages(
   DEPENDENCIES
   geometry_msgs
 )

catkin_package()

add_executable(maze_planner
        src/maze_planner.cpp
        src/occupancy_grid.cpp
        src/node.cpp
        src/open_set.cpp
        src/closed_set.cpp
        src/marker.cpp
        src/path_set.cpp
        )

add_dependencies(maze_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_include_directories(maze_planner SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_include_directories(maze_planner PRIVATE include)

target_link_libraries(maze_planner ${OpenCV_LIBS} ${catkin_LIBRARIES})
