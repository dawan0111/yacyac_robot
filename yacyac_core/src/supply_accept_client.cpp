#include "yacyac_core/supply_accept_client.hpp"

SupplyAcceptClient::SupplyAcceptClient(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode(name, config) {}

SupplyAcceptClient::~SupplyAcceptClient()
{
    RCLCPP_INFO(node_->get_logger(), "SHUTTING DOWN SupplyAcceptClient NODE");
}

BT::NodeStatus SupplyAcceptClient::onStart()
{
    node_ = rclcpp::Node::make_shared("supply_accept_client");

    const std::string QR_topic_name = "/user_feedback";

    has_supply_accept_ = false;

    feedback_sub_ = node_->create_subscription<std_msgs::msg::Bool>(QR_topic_name, rclcpp::QoS(1), [this](const std_msgs::msg::Bool::SharedPtr msg) {
        supply_accept_ = msg->data;
        has_supply_accept_ = true;
    });

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SupplyAcceptClient::onRunning()
{
    rclcpp::spin_some(node_);
    if (has_supply_accept_) {
        if (supply_accept_) {
            return BT::NodeStatus::SUCCESS;
        }
        else {
            return BT::NodeStatus::FAILURE;
        }
    }
    else {
        return BT::NodeStatus::RUNNING;
    }
}

void SupplyAcceptClient::onHalted()
{
    //   printf("[ SupplyAcceptClient: ABORTED ]");
}