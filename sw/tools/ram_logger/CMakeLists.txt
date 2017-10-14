cmake_minimum_required(VERSION 2.8)
project( ram C CXX)

# directories

set (PROJECT_SRC_DIR ./src)
FILE(GLOB MY_SRC_FILES "src/*.cpp" "src/*.h" )

# use sse3 instruction set for libelas and c11 for threading (among others)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3 -std=c++11 -pthread")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3 -std=c++0x -pthread")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

IF(CMAKE_BUILD_TYPE STREQUAL Debug)
        set(DEBUG_FLAG 1)
        ADD_DEFINITIONS(-DDEBUG_FLAG=${DEBUG_FLAG})
        MESSAGE( STATUS "DEBUG MODE" )
ENDIF(CMAKE_BUILD_TYPE STREQUAL Debug)


add_executable( ram ${MY_SRC_FILES} )
