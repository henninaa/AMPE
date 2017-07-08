#
# Project settings
#
project(path_planner CXX)
# Minimum required version of cmake
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Project name and programming languages used
PROJECT( path_planner CXX )
ADD_DEFINITIONS( -std=c++11 )

SET(AMPL_DIRECTORY /home/henning/Documents/prosjektet/ampl/amplapi)

INCLUDE_DIRECTORIES( ${AMPL_DIRECTORY}/include )

file(GLOB SOURCES "source/*.cpp" )

add_executable(pathPlanner ${SOURCES})
add_library(pathPlanner_shared SHARED ${SOURCES})

target_link_libraries(pathPlanner  ${AMPL_DIRECTORY}/lib/libampl.so)
target_link_libraries(pathPlanner_shared  ${AMPL_DIRECTORY}/lib/libampl.so)

SET_TARGET_PROPERTIES( pathPlanner  PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build )

#SET_TARGET_PROPERTIES( pathPlanner_shared  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build )

install(TARGETS pathPlanner_shared DESTINATION /usr/local/lib)
