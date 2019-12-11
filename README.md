# ROS Workspace

[Image Detection](./src/image_detection)

```
git clone git@github.com:yusanshi/ros_package.git --recursive
cd ros_package
export PROJECT_ROOT=`pwd`

cd $PROJECT_ROOT/src/third_party/autoware.ai/visualization
git apply ../../.patches/visualization.patch

cd $PROJECT_ROOT/src/third_party/yolo/weights
bash download_weights.sh
```