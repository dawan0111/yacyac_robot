#ifndef SUPPLY_FEEDBACK_CLIENT_HPP_
#define SUPPLY_FEEDBACK_CLIENT_HPP_

#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class SupplyAcceptClient : public BT::StatefulActionNode {
public:
    SupplyAcceptClient(const std::string& name, const BT::NodeConfig& config);
    ~SupplyAcceptClient();

    static BT::PortsList providedPorts() { return {}; }

    // this function is invoked once at the beginning.
    BT::NodeStatus onStart() override;

    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    BT::NodeStatus onRunning() override;

    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr feedback_sub_;
    bool supply_accept_;
    bool has_supply_accept_;
};

#endif
