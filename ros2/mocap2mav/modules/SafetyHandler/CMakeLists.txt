cmake_minimum_required(VERSION 2.6)
project (SafetyHandler CXX)
#list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")

set (PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#Set QT
##find_package(Qt5Widgets REQUIRED)
#find_package(Qt5Network)

set(M2M_SIM "$ENV{M2M_SIM}")
if(NOT M2M_SIM)
 message(FATAL_ERROR "Undefined M2M_SIM env var. HINT: export M2M_SIM=1 if running simulation, 0 else")
endif()

set(PROJECT_SRCS ${PROJECT_SOURCE_DIR}/main.cpp ${PROJECT_SOURCE_DIR}/SafetyParser.cpp)

include_directories("${PROJECT_INCLUDE_DIR}")
include_directories("libs")
include_directories("include")
include_directories("dir")

add_executable(safety ${PROJECT_SRCS} ${PROJECT_INCLUDE_DIR}/utils/TimeHelpers.hpp)

find_package(PkgConfig REQUIRED)
pkg_search_module(LCM REQUIRED lcm)

##Declare a C++ library
 add_library(Safetylib
         libs/SafetyHandler.cpp
         include/SafetyHandler.h
         ${PROJECT_INCLUDE_DIR}/common/MavState.h
         ${PROJECT_SRC_DIR}/MavState.cpp
         )

target_link_libraries(safety
        	${PROJECT_LINK_LIBS}
		Safetylib
		${LCM_LIBRARIES}
	)

