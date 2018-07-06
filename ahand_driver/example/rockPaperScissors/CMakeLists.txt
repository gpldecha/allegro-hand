
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(rock_paper_scissors src/main.cpp src/RockScissorsPaper.cpp ${INCLUDE_DIR}/rock_paper_scissors/RockScissorsPaper.h)
target_link_libraries(rock_paper_scissors ahand_driver ${catkin_LIBRARIES})
