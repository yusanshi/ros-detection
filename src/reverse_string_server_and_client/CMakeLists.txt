cmake_minimum_required(VERSION 2.8.3)
project(reverse_string_server_and_client)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_service_files(
  FILES
  reverse.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(reverse_string_server src/server.cpp)
target_link_libraries(reverse_string_server ${catkin_LIBRARIES})

add_executable(reverse_string_client src/client.cpp)
target_link_libraries(reverse_string_client ${catkin_LIBRARIES})

add_dependencies(reverse_string_server reverse_string_server_and_client_generate_messages_cpp)
add_dependencies(reverse_string_client reverse_string_server_and_client_generate_messages_cpp)