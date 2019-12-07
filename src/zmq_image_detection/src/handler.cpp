#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <zmq.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"

class ImageDetectionHandler {
   public:
    // TODO: test use raw or compressed image,
    // test data size to transport in a second
    ImageDetectionHandler(std::string image_topic, std::string result_topic,
                          std::string ipc_file_path) {
        it_ = std::unique_ptr<image_transport::ImageTransport>(
            new image_transport::ImageTransport(nh_));
        pub_ = nh_.advertise<std_msgs::String>(result_topic, 1);
        sub_ = it_.get()->subscribe(image_topic, 1,
                                    &ImageDetectionHandler::callback,
                                    this);  // TODO what if remove get()?
        ROS_INFO("Detection result of topic %s is being published to topic %s",
                 image_topic.c_str(), result_topic.c_str());

        zmq_ctx_ = std::unique_ptr<zmq::context_t>(new zmq::context_t(1));
        zmq_sock_ = std::unique_ptr<zmq::socket_t>(
            new zmq::socket_t(*zmq_ctx_, ZMQ_REQ));

        zmq_sock_.get()->connect("ipc://" + ipc_file_path);
    }

    static void custom_free(void* data, void* hint) {
        // Don't "free(data);" here, or seg fault will occur
        // Because this buffer belongs to cv_bridge
        return;
    }

    void callback(const sensor_msgs::ImageConstPtr& msg) {
        auto img = cv_bridge::toCvShare(msg, "rgb8");
        auto img_header = img->header;
        auto img_frame_id = img_header.frame_id;
        auto img_seq = img_header.seq;
        auto img_stamp = img_header.stamp;
        auto img_data = img->image;

        // Some helpful output
        // std::cout << "Row and col:" << img_data.rows << " " << img_data.cols
        //           << std::endl;
        // std::cout << "Pixel count: " << img_data.total() << std::endl;
        // std::cout << "Elem size: " << img_data.elemSize() << std::endl;
        // std::cout << "Total size: " << img_data.total() * img_data.elemSize()
        //           << std::endl;

        zmq::message_t request(
            img_data.data, img_data.total() * img_data.elemSize(), custom_free);
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
    ros::NodeHandle nh_;
    std::unique_ptr<image_transport::ImageTransport> it_;
    ros::Publisher pub_;
    image_transport::Subscriber sub_;
    std::unique_ptr<zmq::context_t> zmq_ctx_;
    std::unique_ptr<zmq::socket_t> zmq_sock_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "zmq_image_detection_handler");
    if (argc != 4) {
        ROS_ERROR(
            "Usage: zmq_image_detection_handler <image topic> \
<result topic> <ipc file path>");
        return 1;
    }

    ImageDetectionHandler handler(std::string{argv[1]}, std::string{argv[2]},
                                  std::string{argv[3]});
    ros::spin();

    return 0;
}