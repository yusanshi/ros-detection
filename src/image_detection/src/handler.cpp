#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <chrono>
#include <opencv2/highgui/highgui.hpp>
#include <zmq.hpp>
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include "json.hpp"
#include "ros/ros.h"

class ImageDetectionHandler {
   public:
    ImageDetectionHandler(std::string image_src_topic) {
        ros_namespace_ = ros::this_node::getNamespace();

        std::string ipc_file_path = ros_namespace_;
        ipc_file_path.erase(
            std::remove(ipc_file_path.begin(), ipc_file_path.end(), '/'),
            ipc_file_path.end());
        ipc_file_path = "/tmp/" + ipc_file_path + ".ipc";

        if (ros_namespace_.substr(0, 2) == "//") {
            ros_namespace_.erase(ros_namespace_.begin());
        }

        if (image_src_topic.substr(0, 1) != "/") {
            image_src_topic = "/" + image_src_topic;
        }

        image_src_topic = ros_namespace_ + image_src_topic;
        std::string detection_output_topic = ros_namespace_ + "/objects";

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
        std::chrono::steady_clock::time_point begin =
            std::chrono::steady_clock::now();

        nlohmann::json first_part_json = {{"height", msg->height},
                                          {"width", msg->width}};
        std::string first_part_str = first_part_json.dump();

        zmq::message_t first_part(first_part_str.size());
        std::memcpy(first_part.data(), first_part_str.c_str(),
                    first_part_str.size());
        zmq_sock_.get()->send(first_part, ZMQ_SNDMORE);

        // TODO toCvShare or toCvCopy?
        cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(msg, "rgb8");
        cv::Mat mat_image = cv_image->image;
        zmq::message_t second_part(mat_image.data,
                                   mat_image.total() * mat_image.elemSize(),
                                   dummy_free);
        zmq_sock_.get()->send(second_part, 0);

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
        std::chrono::steady_clock::time_point end =
            std::chrono::steady_clock::now();
        ROS_INFO(
            "Time elapsed: "
            "%f",
            std::chrono::duration_cast<std::chrono::microseconds>(end - begin)
                    .count() /
                1000.0);
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
    ros::init(argc, argv, "image_detection_handler");
    if (argc != 2) {
        ROS_ERROR("Usage: image_detection_handler <image topic>");
        return 1;
    }

    ImageDetectionHandler handler(std::string{argv[1]});
    ros::spin();

    return 0;
}