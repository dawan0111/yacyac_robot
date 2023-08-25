#include "yacyac_core/qr_client.hpp"

QRClient::QRClient(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("qr_client");
    std::string QR_topic_name = "/qr_codes";
    node_->create_subscription<yacyac_interface::msg::Qrcode>(QR_topic_name, rclcpp::QoS(1), std::bind(&QRClient::QR_callback_, this, std::placeholders::_1));
}

QRClient::~QRClient()
{
    RCLCPP_INFO(node_->get_logger(), "SHUTTING DOWN QR NODE");
}

BT::NodeStatus QRClient::onStart()
{
    double life_time = rclcpp::Duration(10, 0).seconds();
    deadline_ = node_->get_clock()->now().seconds() + life_time;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus QRClient::onRunning()
{
    // rclcpp::spin_some(node_);
    if (node_->get_clock()->now().seconds() <= deadline_) {
        int8_t detected_QR_length = detected_QR_.size();
        RCLCPP_INFO(node_->get_logger(), "detected qr length: %d", detected_QR_length);

        if (detected_QR_length == 1) {
            return BT::NodeStatus::SUCCESS;
        }
        else if (detected_QR_length > 1) {
            // TODO: TTS
        }

        return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::FAILURE;
}

void QRClient::onHalted()
{
    //   printf("[ QRClient: ABORTED ]");
}

void QRClient::QR_callback_(const yacyac_interface::msg::Qrcode::ConstSharedPtr& msg)
{
    detected_QR_ = msg->qr_infos;
}