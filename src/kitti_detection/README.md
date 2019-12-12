

```
AUTOWARE_COMPILE_WITH_CUDA=1 catkin_make --force-cmake --only-pkg-with-deps detected_objects_visualizer range_vision_fusion lidar_point_pillars vision_darknet_detect

# For rosbag from KITTI dataset by [kitti2bag](https://github.com/valgur/kitti2bag)
cd /path/to/project_root
PROJECT_ROOT=`pwd` roslaunch kitti_detection autoware_detection.launch
```