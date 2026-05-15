cmake_minimum_required(VERSION 2.6)
project (miniPID CXX)
set(CMAKE_CXX_STANDARD 11)

# Set build directory
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

#Set QT
##find_package(Qt5Widgets REQUIRED)
#find_package(Qt5Network)

set(PROJECT_SRCS
	MiniPID.cpp
	MiniPID.h
)

add_executable(pidExampleCPP examples/GeneralCPP/main.cpp)

##Declare a C++ library
add_library(miniPID ${PROJECT_SRCS})

target_link_libraries(pidExampleCPP miniPID)
