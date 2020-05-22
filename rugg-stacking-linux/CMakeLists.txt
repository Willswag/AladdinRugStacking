cmake_minimum_required(VERSION 3.10)

# set the project name
project(vision-server VERSION 3.0)

## use this to globally use C++11 with in our project
set(CMAKE_CXX_STANDARD 11)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

#find cppzmq wrapper, installed by make of cppzmq
find_package(cppzmq)


# add the executable
add_executable(vision-server vision-server.cpp)
target_link_libraries( vision-server ${OpenCV_LIBS} cppzmq )


