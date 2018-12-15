cmake_minimum_required(VERSION 2.8.3)
project(test_kdl)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  kdl_parser
)

add_definitions(-std=c++11)
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp kdl_parser
  # INCLUDE_DIRS include
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_kdl_cpp src/test_kdl.cpp)
target_link_libraries(test_kdl_cpp ${catkin_LIBRARIES})
add_dependencies(test_kdl_cpp ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
