cmake_minimum_required(VERSION 2.8.3)
project(zmq_image_detection)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(zmq_image_detection_handler src/handler.cpp)
target_link_libraries(zmq_image_detection_handler ${catkin_LIBRARIES})
target_link_libraries(zmq_image_detection_handler -lzmq) # Link ZMQ
target_link_libraries(zmq_image_detection_handler ${OpenCV_LIBS}) # Link OpenCV


