# LINKE Detection

```
cd /path/to/project_root
AUTOWARE_COMPILE_WITH_CUDA=1 catkin_make --force-cmake --only-pkg-with-deps detected_objects_visualizer range_vision_fusion lidar_point_pillars vision_darknet_detect calibration_publisher image_processor

cd /path/to/project_root/src/third_party/autoware.ai/core_perception/vision_darknet_detect/darknet/data
bash download_weights.sh

cd /path/to/project_root
PROJECT_ROOT=`pwd` roslaunch linke_detection detection.launch
```