cmake_minimum_required(VERSION 2.8.3)
project(image_detection)

add_compile_options(-std=c++11)
#list(APPEND CMAKE_PREFIX_PATH "/home/yu/catkin_ws/src/third_party/nlohmann")
find_package(autoware_msgs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  autoware_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

#find_package(nlohmann_json REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  cv_bridge
  roscpp
  sensor_msgs
  autoware_msgs
  std_msgs
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_detection_handler src/handler.cpp)
target_link_libraries(image_detection_handler 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} # Link OpenCV
  -lzmq # Link ZMQ
  #nlohmann_json::nlohmann_json
)
add_dependencies(image_detection_handler autoware_msgs_generate_messages_cpp)