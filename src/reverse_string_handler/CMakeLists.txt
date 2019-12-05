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

add_executable(reverse_string_publisher src/publisher.cpp)
target_link_libraries(reverse_string_publisher ${catkin_LIBRARIES})

add_executable(reverse_string_subscriber src/subscriber.cpp)
target_link_libraries(reverse_string_subscriber ${catkin_LIBRARIES})