# ROS Workspace

[ZMQ Image Detection](./src/zmq_image_detection)

```
catkin_make --force-cmake --only-pkg-with-deps zmq_image_detection detected_objects_visualizer

cd /path/to/launch
ROS_NAMESPACE=/usb_cam roslaunch image_detection.launch
cd /path/to/src/zmq_server
ROS_NAMESPACE=/usb_cam python3.6 server.py


cd /path/to/launch
ROS_NAMESPACE=/usb_cam2 roslaunch image_detection.launch
cd /path/to/src/zmq_server
ROS_NAMESPACE=/usb_cam2 python3.6 server.py

......

```

> How to restore to build all packages after `--only-pkg-with-deps`?  
Use `catkin_make -DCATKIN_WHITELIST_PACKAGES=""`.