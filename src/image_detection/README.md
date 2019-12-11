# ZMQ Image Detection

```
cd /path/to/project_root
catkin_make --force-cmake --only-pkg-with-deps image_detection detected_objects_visualizer

# Configurate
vim src/image_detection/src/server/config.py

# New terminal
cd /path/to/project_root
ROS_NAMESPACE=/kitti/camera_color/left \
roslaunch src/image_detection/launch/image_detection.launch \
image_src_topic:=image_rect_color

# New terminal
cd /path/to/project_root
ROS_NAMESPACE=/kitti/camera_color/left \
python3.6 src/image_detection/src/server/server.py

# Results in topic: ROS_NAMESPACE/image_rects
# Use Rviz to visualize.
```

> How to restore to build all packages after `--only-pkg-with-deps`?  
Use `catkin_make -DCATKIN_WHITELIST_PACKAGES=""`.


![](https://img.yusanshi.com/upload/20191209004317261636.png)