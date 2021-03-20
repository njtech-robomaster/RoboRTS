cmake_minimum_required(VERSION 3.10)

project(armor_detector)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs
)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

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
	${OpenCV_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_executable(armor_detector ${cxx_src})
target_link_libraries(armor_detector
	PUBLIC
	${OpenCV_LIBS}
	${realsense2_LIBRARY}
	${catkin_LIBRARIES}
)
