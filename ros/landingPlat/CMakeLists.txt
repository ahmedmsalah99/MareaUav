cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
PROJECT(landingPlat CXX)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

include_directories(include)

set(M2M_INCLUDE "$ENV{M2M_INCLUDE}")
if(M2M_INCLUDE)
    include_directories(${M2M_INCLUDE})
else()
    message(STATUS "INCLUDE at ${M2M_INCLUDE}")
    message(FATAL_ERROR "Undefined M2M_INCLUDE env var. HINT: export M2M_INCLUDE=/path/to/mocap2mav/include")
endif()

find_package(PkgConfig REQUIRED)
pkg_search_module(LCM REQUIRED lcm)

add_library(platformPlugin SHARED src/plugin.cpp src/WaveGen.cpp include/WaveGen.h)
target_link_libraries(platformPlugin
        ${GAZEBO_LIBRARIES}
        ${LCM_LIBRARIES})
