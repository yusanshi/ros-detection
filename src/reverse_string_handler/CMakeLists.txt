cmake_minimum_required(VERSION 2.8.3)
project(reverse_string_handler)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(reverse_string_handler src/handler.cpp)
target_link_libraries(reverse_string_handler ${catkin_LIBRARIES})

add_executable(string_publisher src/publisher.cpp)
target_link_libraries(string_publisher ${catkin_LIBRARIES})

add_executable(string_listener src/listener.cpp)
target_link_libraries(string_listener ${catkin_LIBRARIES})