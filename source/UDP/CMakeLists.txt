cmake_minimum_required(VERSION 3.0)

#add_definitions( -std=c++11 )
#add_definitions( -lboost_system )

SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

file(GLOB SOURCES1 "source/*.cpp" )
file(GLOB SOURCES2 "sender/*.cpp" )
file(GLOB SOURCES3 "receiver/*.cpp" )

find_package(Boost 1.58.0 COMPONENTS system thread )
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  add_executable( UDP_peer ${SOURCES1})
  target_link_libraries(UDP_peer ${Boost_LIBRARIES})
  SET_TARGET_PROPERTIES( UDP_peer  PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build )

  add_executable( receiver ${SOURCES3})
  target_link_libraries(receiver ${Boost_LIBRARIES})
  SET_TARGET_PROPERTIES( receiver  PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build )

  add_executable( sender ${SOURCES2})
  target_link_libraries(sender ${Boost_LIBRARIES})
  SET_TARGET_PROPERTIES( sender  PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build )


endif()

