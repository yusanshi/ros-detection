# ROS Workspace

[ZMQ Image Detection](./src/zmq_image_detection)

```
git clone git@github.com:yusanshi/ros_package.git --recursive

cd /path/to/src/autoware.ai/visualization
git apply ../visualization.patch

cd /path/to/src/zmq_image_detection/src/zmq_server/yolo_e1
git apply ../yolo_e1.patch

cd /path/to/src/zmq_image_detection/src/zmq_server/yolo_e1/weights
bash download_weights.sh

cd /path/to/ros_package
catkin_make --force-cmake --only-pkg-with-deps zmq_image_detection detected_objects_visualizer

cd /path/to/launch
ROS_NAMESPACE=/usb_cam roslaunch image_detection.launch
cd /path/to/src/zmq_server
ROS_NAMESPACE=/usb_cam python3.6 server.py

cd /path/to/launch
ROS_NAMESPACE=/usb_cam2 roslaunch image_detection.launch
cd /path/to/src/zmq_server
ROS_NAMESPACE=/usb_cam2 python3.6 server.py


# Results in topic: ROS_NAMESPACE/image_rects
# Use Rviz to visualize.

......

```

> How to restore to build all packages after `--only-pkg-with-deps`?  
Use `catkin_make -DCATKIN_WHITELIST_PACKAGES=""`.


![](https://img.yusanshi.com/upload/20191209004317261636.png)