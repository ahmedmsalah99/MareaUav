cmake_minimum_required(VERSION 2.6)
project(StateMachine CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

include_directories(include)

#Create lib
set(SOURCE_FILES  src/Machine.cpp include/Machine.h src/AbstractState.cpp include/AbstractState.h)
add_library(StateMachine ${SOURCE_FILES})

#Create Example executable
add_executable(example src/main.cpp)
target_link_libraries(example StateMachine)