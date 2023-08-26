#ifndef QR_DETECTOR_NODE_HPP_
#define QR_DETECTOR_NODE_HPP_
#include "yacyac_interface/msg/qrcode.hpp"
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <zbar.h>

class QrDetectorNode final : public rclcpp::Node {
public:
    explicit QrDetectorNode(const std::string& node_name, const std::string& input_topic_name, const std::string& output) : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        RCLCPP_INFO(this->get_logger(), "QR DETECTOR NODE CREATE");
        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(input_topic_name, rclcpp::SensorDataQoS(), std::bind(&QrDetectorNode::image_callback_, this, std::placeholders::_1));

        publisher_ = this->create_publisher<yacyac_interface::msg::Qrcode>(output, rclcpp::QoS(1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&QrDetectorNode::publish_qr_, this));
    }

private:
    void publish_qr_()
    {
        yacyac_interface::msg::Qrcode message;
        message.qr_infos = qr_detect_infos_;

        publisher_->publish(message);
    }

    void image_callback_(sensor_msgs::msg::Image::UniquePtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {

            cv_ptr = cv_bridge::toCvCopy(std::move(msg), msg->encoding);
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        qr_detect_infos_ = getQRcodeInfo(cv_ptr->image);
    }

    std::vector<std::string> getQRcodeInfo(const cv::Mat& input_image)
    {
        cv::Mat gray_image;
        cv::cvtColor(input_image, gray_image, CV_BGR2GRAY);
        std::vector<std::string> qr_infos;

        const auto width = input_image.cols;
        const auto height = input_image.rows;

        zbar::Image img(width, height, "Y800", gray_image.data, width * height);
        scanner_.scan(img);

        for (auto s = img.symbol_begin(); s != img.symbol_end(); ++s) {
            std::string qr_info = s->get_data();
            qr_infos.push_back(qr_info);
        }

        return qr_infos;
    };
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<yacyac_interface::msg::Qrcode>::SharedPtr publisher_;

    zbar::ImageScanner scanner_;

    std::vector<std::string> qr_detect_infos_;
};

#endif