#include "ros/ros.h"
#include "std_msgs/String.h"

void subscribe_callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received: %s", msg->data.c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "string_listener");

    ros::NodeHandle n;

    ros::Subscriber sub =
        n.subscribe("/reversed_string", 1000, subscribe_callback);

    ros::spin();

    return 0;
}
