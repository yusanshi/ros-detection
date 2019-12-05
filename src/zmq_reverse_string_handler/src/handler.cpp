#include <zmq.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"

class ReverseStringHandler {
   public:
    ReverseStringHandler() {
        pub_ = n_.advertise<std_msgs::String>("/reversed_string", 1);
        sub_ = n_.subscribe("/original_string", 1,
                            &ReverseStringHandler::callback, this);
        zmq_ctx_ = std::unique_ptr<zmq::context_t>(new zmq::context_t(1));
        zmq_sock_ = std::unique_ptr<zmq::socket_t>(
            new zmq::socket_t(*zmq_ctx_, ZMQ_REQ));        // TODO
        zmq_sock_.get()->connect("ipc:///tmp/reverse_string.ipc");  // TODO
    }

    void callback(const std_msgs::String::ConstPtr& input) {
        std::string temp = input->data;
        zmq::message_t request(temp.size());
        std::memcpy(request.data(), temp.c_str(), temp.size());
        ROS_INFO("Send to ZMQ: %s", temp.c_str());
        zmq_sock_.get()->send(request);

        zmq::message_t reply;
        zmq_sock_.get()->recv(&reply);

        std_msgs::String result;
        result.data =
            std::string(static_cast<char*>(reply.data()), reply.size());
        ROS_INFO("Receive from ZMQ: %s", result.data.c_str());
        pub_.publish(result);
    }

   private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    std::unique_ptr<zmq::context_t> zmq_ctx_;
    std::unique_ptr<zmq::socket_t> zmq_sock_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "reverse_string_handler");
    ReverseStringHandler handler;
    ros::spin();

    return 0;
}