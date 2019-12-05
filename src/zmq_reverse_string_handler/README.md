# ZMQ Reverse String Handler

```bash
# cd into current workspace
catkin_make --force-cmake

# Open a new terminal
rosrun zmq_reverse_string_handler zmq_reverse_string_handler

# Open another terminal
rosrun zmq_reverse_string_handler zmq_reverse_string_subscriber

# Open another terminal again
rosrun zmq_reverse_string_handler zmq_reverse_string_publisher

# Open another terminal again
python3 ./zmq_reverse_string_handler/src/zmq_server.py
```
