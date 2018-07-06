cmake_minimum_required(VERSION 2.8.3)
project(can_api)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES can_api
  CATKIN_DEPENDS
  DEPENDS pcanbasic
)

###########
## Build ##
###########

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)


include_directories(${catkin_INCLUDE_DIRS} include)

add_library(can_api src/canAPI.cpp ${INCLUDE_DIR}/can_api/canAPI.h ${INCLUDE_DIR}/can_api/canDef.h)
target_link_libraries(can_api pcanbasic)
