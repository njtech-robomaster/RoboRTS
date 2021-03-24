cmake_minimum_required(VERSION 3.10)

project(shoot_controller)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	nav_msgs
	tf2_ros
	tf2_geometry_msgs
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

file(GLOB_RECURSE cxx_src "src/*.h" "src/*.hpp" "src/*.cpp")

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(shoot_controller ${cxx_src})
target_link_libraries(shoot_controller
	PUBLIC
	${catkin_LIBRARIES}
)
