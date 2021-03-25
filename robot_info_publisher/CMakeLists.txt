cmake_minimum_required(VERSION 3.10)

project(robot_info_publisher)

find_package(catkin REQUIRED COMPONENTS
	roscpp
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
add_executable(robot_info_publisher ${cxx_src})
target_link_libraries(robot_info_publisher
	PUBLIC
	${catkin_LIBRARIES}
)
add_dependencies(robot_info_publisher
  roborts_msgs_generate_messages)
