#include <algorithm>
#include "ros/ros.h"
#include "std_msgs/String.h"

std::string random_string(size_t length) {
    auto randchar = []() -> char {
        const char charset[] =
            "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
        const size_t max_index = (sizeof(charset) - 1);
        return charset[rand() % max_index];
    };
    std::string str(length, 0);
    std::generate_n(str.begin(), length, randchar);
    return str;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "string_publisher");

    ros::NodeHandle n;

    ros::Publisher chatter_pub =
        n.advertise<std_msgs::String>("/original_string", 1000);

    ros::Rate loop_rate(2);

    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = random_string(10);
        ROS_INFO("Send: %s", msg.data.c_str());
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}