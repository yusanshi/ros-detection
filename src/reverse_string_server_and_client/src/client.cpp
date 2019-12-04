#include "reverse_string_server_and_client/reverse.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "reverse_string_client");
    if (argc != 2) {
        ROS_INFO("Usage: reverse_string_client X");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client =
        n.serviceClient<reverse_string_server_and_client::reverse>(
            "reverse_string");
    reverse_string_server_and_client::reverse srv;
    srv.request.original = argv[1];
    if (client.call(srv)) {
        ROS_INFO("Original: %s", srv.request.original.c_str());
        ROS_INFO("Reversed: %s", srv.response.reversed.c_str());
        return 0;
    } else {
        ROS_ERROR("Failed to call servie reverse");
        return 1;
    }
}