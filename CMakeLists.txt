cmake_minimum_required(VERSION 3.0.0)
project(PlyTest VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(PlyTest src/main.cpp)
target_link_libraries(PlyTest fmt::fmt)
target_link_libraries( PlyTest ${OpenCV_LIBS} )