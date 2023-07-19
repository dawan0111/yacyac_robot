#ifndef CAMERA_NODE_HPP_
#define CAMERA_NODE_HPP_

#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

/// Node which captures images from a camera using OpenCV and publishes them.
/// Images are annotated with this process's id as well as the message's ptr.
class CameraNode final : public rclcpp::Node {
public:
    explicit CameraNode(const std::string& output, const std::string& node_name = "camera_node", const std::string& device = "/dev/video0", int width = 640, int height = 480)
        : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)), canceled_(false)
    {
        // Initialize OpenCV
        cap_.open(device);

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));

        if (!cap_.isOpened()) {
            throw std::runtime_error("Could not open video stream!");
        }
        // Create a publisher on the output topic.
        pub_ = this->create_publisher<sensor_msgs::msg::Image>(output, rclcpp::SensorDataQoS());
        thread_ = std::thread(std::bind(&CameraNode::loop, this));
    }

    ~CameraNode()
    {
        canceled_.store(true);
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    void loop()
    {
        while (rclcpp::ok() && !canceled_.load()) {
            cap_ >> frame_;
            if (frame_.empty()) {
                continue;
            }

            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_).toImageMsg();
            pub_->publish(*msg); // Publish.
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    std::thread thread_;
    std::atomic<bool> canceled_;
    bool watermark_;

    cv::VideoCapture cap_;
    cv::Mat frame_;
};

#endif