cmake_minimum_required(VERSION 3.10)

project(simple_decision)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	nav_msgs
	tf2_ros
	tf2_geometry_msgs
	actionlib
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

file(GLOB_RECURSE cxx_src
	"src/*.h"
	"src/*.hpp"
	"src/*.cpp"
)
add_executable(simple_decision ${cxx_src})
target_link_libraries(simple_decision
	PUBLIC
	${catkin_LIBRARIES}
)
add_dependencies(simple_decision
  roborts_msgs_generate_messages)
