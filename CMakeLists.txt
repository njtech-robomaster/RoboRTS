cmake_minimum_required(VERSION 3.10)

project(armor_detect)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB_RECURSE cxx_src "src/*.h" "src/*.cpp")

add_executable(armor_detect ${cxx_src})
target_link_libraries(armor_detect
	PUBLIC
	${OpenCV_LIBS}
)
