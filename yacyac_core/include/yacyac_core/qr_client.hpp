#ifndef QR_CLIENT_HPP_
#define QR_CLIENT_HPP_

#include <behaviortree_cpp/behavior_tree.h>
#include <std_msgs/msg/int8.hpp>
#include <rclcpp/rclcpp.hpp>

#include "yacyac_interface/msg/qrcode.hpp"

class QRClient : public BT::StatefulActionNode {
public:
    QRClient(const std::string& name, const BT::NodeConfig& config);
    ~QRClient();

    static BT::PortsList providedPorts() { return {}; }

    // this function is invoked once at the beginning.
    BT::NodeStatus onStart() override;

    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    BT::NodeStatus onRunning() override;

    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<yacyac_interface::msg::Qrcode>::SharedPtr qr_sub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mode_publisher_;
    double deadline_;
    std::vector<std::string> detected_QR_;

    void QR_callback_(const yacyac_interface::msg::Qrcode::ConstSharedPtr& msg);
};

#endif
