#include "reverse_string_server_and_client/reverse.h"
#include "ros/ros.h"

bool reverse(reverse_string_server_and_client::reverse::Request &req,
             reverse_string_server_and_client::reverse::Response &res) {
    std::string temp = req.original;
    std::reverse(temp.begin(), temp.end());
    res.reversed = temp;
    ROS_INFO("Original: %s", req.original.c_str());
    ROS_INFO("Reversed: %s", res.reversed.c_str());
    std::cout << "\n";

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "reverse_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("reverse_string", reverse);
    ROS_INFO("Ready to reverse.");
    ros::spin();

    return 0;
}