# Image Detection

```
cd /path/to/project_root
catkin_make --force-cmake --only-pkg-with-deps image_detection detected_objects_visualizer

cd /path/to/project_root/src/third_party/yolo/weights
bash download_weights.sh

# Configurate
vim src/image_detection/src/server/config.py

# New terminal

IPC_FILE_PATH=/tmp/image_detection.ipc \
roslaunch image_detection image_detection.launch

# New terminal
cd /path/to/project_root
IPC_FILE_PATH=/tmp/image_detection.ipc \
python3.6 src/image_detection/src/server/server.py

# Results in topic: /detection/image_rects
# Use Rviz to visualize.
```

> How to restore to build all packages after `--only-pkg-with-deps`?  
Use `catkin_make -DCATKIN_WHITELIST_PACKAGES=""`.


![](https://img.yusanshi.com/upload/20191209004317261636.png)