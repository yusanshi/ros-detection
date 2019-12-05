#include "ros/ros.h"
#include "std_msgs/String.h"

class ReverseStringHandler {
   public:
    ReverseStringHandler() {
        pub = n.advertise<std_msgs::String>("/reversed_string", 1);
        sub = n.subscribe("/original_string", 1,
                          &ReverseStringHandler::callback, this);
    }

    void callback(const std_msgs::String::ConstPtr& input) {
        std::string temp = input->data;
        std::reverse(temp.begin(), temp.end());
        std_msgs::String output;
        output.data = temp;
        pub.publish(output);
    }

   private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "reverse_string_handler");
    ReverseStringHandler handler;
    ros::spin();

    return 0;
}