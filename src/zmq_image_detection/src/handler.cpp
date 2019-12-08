#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>

#include <opencv2/highgui/highgui.hpp>
#include <zmq.hpp>
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include "ros/ros.h"
#include "zmq_image_detection/json.hpp"

class ImageDetectionHandler {
   public:
    // TODO: test use raw or compressed image,
    // test data size to transport in a second
    ImageDetectionHandler(std::string image_src_topic) {
        ros_namespace_ = ros::this_node::getNamespace();
        if (ros_namespace_.substr(0, 2) == "//") {
            ros_namespace_.erase(ros_namespace_.begin());
        }
        if (image_src_topic.substr(0, 1) != "/") {
            image_src_topic = "/" + image_src_topic;
        }

        image_src_topic = ros_namespace_ + image_src_topic;
        std::string detection_output_topic = ros_namespace_ + "/objects";
        std::string ipc_file_path = "/tmp" + ros_namespace_ + ".ipc";

        sub_ = nh_.subscribe(image_src_topic, 1,
                             &ImageDetectionHandler::callback, this);
        pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>(
            detection_output_topic, 1);

        zmq_ctx_ = std::unique_ptr<zmq::context_t>(new zmq::context_t(1));
        zmq_sock_ = std::unique_ptr<zmq::socket_t>(
            new zmq::socket_t(*zmq_ctx_, ZMQ_REQ));

        zmq_sock_.get()->connect("ipc://" +
                                 ipc_file_path);  // TODO what if remove get()?

        ROS_INFO(
            "Detection result of topic %s is being published to topic %s, "
            "using ZMQ ipc://%s",
            image_src_topic.c_str(), detection_output_topic.c_str(),
            ipc_file_path.c_str());
    }

    static void dummy_free(void* data, void* hint) {
        // Don't "free(data);" here, or seg fault will occur
        // Because this buffer belongs to cv_bridge
        return;
    }

    void callback(const sensor_msgs::ImageConstPtr& msg) {
        // TODO toCvShare or toCvCopy?
        cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(msg, "rgb8");
        cv::Mat mat_image = cv_image->image;

        zmq::message_t request(mat_image.data,
                               mat_image.total() * mat_image.elemSize(),
                               dummy_free);
        zmq_sock_.get()->send(request);

        zmq::message_t reply;
        zmq_sock_.get()->recv(&reply);
        std::string reply_string =
            std::string(static_cast<char*>(reply.data()), reply.size());

        autoware_msgs::DetectedObjectArray result;
        result.header = msg->header;

        auto reply_json = nlohmann::json::parse(reply_string);
        for (auto e : reply_json) {
            autoware_msgs::DetectedObject obj;
            obj.x = e["x"];
            obj.y = e["y"];
            obj.width = e["width"];
            obj.height = e["height"];
            obj.score = e["score"];
            obj.label = e["label"];
            obj.valid = true;
            result.objects.push_back(obj);
        }
        pub_.publish(result);
    }

   private:
    std::string ros_namespace_;
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    std::unique_ptr<zmq::context_t> zmq_ctx_;
    std::unique_ptr<zmq::socket_t> zmq_sock_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "zmq_image_detection_handler");
    if (argc != 2) {
        ROS_ERROR("Usage: zmq_image_detection_handler <image topic>");
        return 1;
    }

    ImageDetectionHandler handler(std::string{argv[1]});
    ros::spin();

    return 0;
}