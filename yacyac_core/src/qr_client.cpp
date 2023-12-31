#include "yacyac_core/qr_client.hpp"

QRClient::QRClient(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode(name, config) {}

QRClient::~QRClient()
{
    RCLCPP_INFO(node_->get_logger(), "SHUTTING DOWN QR NODE");
}

BT::NodeStatus QRClient::onStart()
{
    node_ = rclcpp::Node::make_shared("qr_client");

    const std::string QR_topic_name = "/qr_node";
    const std::string mode_topic_name = "/mode";
    const std::string TTS_service_name = "/yacyac/io"; 

    qr_sub_ = node_->create_subscription<yacyac_interface::msg::Qrcode>(QR_topic_name, rclcpp::QoS(1), std::bind(&QRClient::QR_callback_, this, std::placeholders::_1));
    mode_publisher_ = node_->create_publisher<std_msgs::msg::Int8>(mode_topic_name, rclcpp::QoS(1));
    TTS_client_ = node_->create_client<yacyac_interface::srv::TTS>(TTS_service_name);

    double life_time = rclcpp::Duration(30, 0).seconds();
    deadline_ = node_->get_clock()->now().seconds() + life_time;

    auto message = std_msgs::msg::Int8();
    message.data = 1;
    mode_publisher_->publish(message);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus QRClient::onRunning()
{
    rclcpp::spin_some(node_);
    if (node_->get_clock()->now().seconds() <= deadline_) {
        int8_t detected_QR_length = detected_QR_.size();
        // RCLCPP_INFO(node_->get_logger(), "detected qr length: %d", detected_QR_length);

        if (detected_QR_length == 1) {
            setOutput("user_id", detected_QR_[0]);
            auto request = std::make_shared<yacyac_interface::srv::TTS::Request>();
            request->tts_str_t = "QR코드가 인식되었습니다.";
            auto result = TTS_client_->async_send_request(request);
            return BT::NodeStatus::SUCCESS;
        }
        else if (detected_QR_length > 1 && prev_QR_length_ <= 1) {
            auto request = std::make_shared<yacyac_interface::srv::TTS::Request>();
            request->tts_str_t = "QR코드가 여러개 인식되었습니다. 다시 인식해주세요.";
            auto result = TTS_client_->async_send_request(request);
        }

        prev_QR_length_ = detected_QR_length;

        return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::FAILURE;
}

void QRClient::onHalted()
{
    //   printf("[ QRClient: ABORTED ]");
}

void QRClient::QR_callback_(const yacyac_interface::msg::Qrcode& msg)
{
    // RCLCPP_INFO(node_->get_logger(), "detected qr length: %d", msg.qr_infos.size());
    detected_QR_ = msg.qr_infos;
}