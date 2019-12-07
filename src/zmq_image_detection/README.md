# ZMQ Image Detection

```bash
# cd into current workspace
catkin_make --force-cmake

# Play a rosbag which contains image topic
rosbag play /path/to/rosbag.bag

# Open a new terminal
rosrun zmq_image_detection zmq_image_detection_handler <image topic> <result topic> <ipc file path>

# Open another terminal
# cd into current workspace
python3 ./src/zmq_image_detection/src/zmq_server/server.py <ipc file path>
```
