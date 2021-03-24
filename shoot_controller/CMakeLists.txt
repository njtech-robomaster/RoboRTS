cmake_minimum_required(VERSION 3.10)

project(shoot_controller)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	nav_msgs
	tf2_ros
	tf2_geometry_msgs
	roborts_msgs
)

catkin_package()

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(
	${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE controller_cxx_src
	"src/controller/*.h"
	"src/controller/*.hpp"
	"src/controller/*.cpp"
)
add_executable(shoot_controller ${controller_cxx_src})
target_link_libraries(shoot_controller
	PUBLIC
	${catkin_LIBRARIES}
)
add_dependencies(shoot_controller
  roborts_msgs_generate_messages)

file(GLOB_RECURSE visualization_cxx_src
	"src/visualization/*.h"
	"src/visualization/*.hpp"
	"src/visualization/*.cpp"
)
add_executable(trajectory_visualization ${visualization_cxx_src})
target_link_libraries(trajectory_visualization
	PUBLIC
	${catkin_LIBRARIES}
)
add_dependencies(trajectory_visualization
  roborts_msgs_generate_messages)
