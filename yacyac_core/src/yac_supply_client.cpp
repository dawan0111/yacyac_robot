// // #include "yacyac_core/nav2_clinent.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "std_msgs/msg/header.hpp"
#include "yacyac_interface/action/supply.hpp"
#include "yacyac_interface/srv/tts.hpp"

// Custom type
struct YacSupplyList {
    int yac_supply_list[8];
};

namespace BT {
template <>
inline YacSupplyList convertFromString(StringView key)
{
    // three real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 8) {
        throw BT::RuntimeError("invalid input)");
    }
    else {
        YacSupplyList output;

        for (int i = 0; i < 8; i++) {
            output.yac_supply_list[i] = convertFromString<uint>(parts[i]);
        }
        std::cout << "yac supply list" << std::endl;
        for (int i = 0; i < 8; i++) {
            std::cout << output.yac_supply_list[i] << " ";
        }
        std::cout << std::endl;
        return output;
    }
}
} // namespace BT
// SyncActionNode

// end namespace BT

class YacSupplyCilent : public BT::SyncActionNode {
public:
    YacSupplyCilent(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config) { std::cout << "yac supplyt client start" << std::endl; }

    static BT::PortsList providedPorts() { return { BT::InputPort<YacSupplyList>("goal") }; }

    virtual BT::NodeStatus tick() override
    {

        std::cout << "yac supply client tick" << std::endl;
        node_ = rclcpp::Node::make_shared("yac_supply_client");
        TTS_client_ = node_->create_client<yacyac_interface::srv::TTS>(TTS_service_name);

        auto action_client = rclcpp_action::create_client<yacyac_interface::action::Supply>(node_, "yacyac/supply_action");
        // if no server is present, fail after 5 seconds
        std::cout << "yac supply server wait" << std::endl;
        if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
            RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
            return BT::NodeStatus::FAILURE;
        }
        // Take the goal from the InputPort of the Node
        YacSupplyList goal;
        if (!getInput<YacSupplyList>("goal", goal)) {
            // if I can't get this, there is something wrong with your BT.
            // For this reason throw an exception instead of returning FAILURE
            throw BT::RuntimeError("missing required input [goal]");
        }

        _aborted = false;

        RCLCPP_INFO(node_->get_logger(), "Yac supply Sending goal %d %d %d %d %d %d %d %d", goal.yac_supply_list[0], goal.yac_supply_list[1], goal.yac_supply_list[2], goal.yac_supply_list[3],
                    goal.yac_supply_list[4], goal.yac_supply_list[5], goal.yac_supply_list[6], goal.yac_supply_list[7]);

        // #include "yacyac_interface/action/supply.hpp"

        auto goal_msg = yacyac_interface::action::Supply::Goal();
        // yacyac_interface::action::Supply goal_msg;
        //  msg->qr_infos
        auto request = std::make_shared<yacyac_interface::srv::TTS::Request>();
        request->tts_str_t = "약 제조중입니다";
        auto result = TTS_client_->async_send_request(request);
        for (int i = 0; i < 8; i++) {
            goal_msg.yac_supply_list[i] = (goal.yac_supply_list[i]);
            std::cout << goal_msg.yac_supply_list[i] << " ";
        }
        std::cout << std::endl;
        // // goal_msg.yac_supply_list[0] = 8;

        auto goal_handle_future = action_client->async_send_goal(goal_msg);
        if (rclcpp::spin_until_future_complete(node_, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "send goal call failed");
            return BT::NodeStatus::FAILURE;
        }

        rclcpp_action::ClientGoalHandle<yacyac_interface::action::Supply>::SharedPtr goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
            return BT::NodeStatus::FAILURE;
        }

        auto result_future = action_client->async_get_result(goal_handle);

        RCLCPP_INFO(node_->get_logger(), "Waiting for result");
        if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "get result call failed ");
            return BT::NodeStatus::FAILURE;
        }

        rclcpp_action::ClientGoalHandle<yacyac_interface::action::Supply>::WrappedResult wrapped_result = result_future.get();

        switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: break;
        case rclcpp_action::ResultCode::ABORTED: RCLCPP_ERROR(node_->get_logger(), "Goal was aborted"); return BT::NodeStatus::FAILURE;
        case rclcpp_action::ResultCode::CANCELED: RCLCPP_ERROR(node_->get_logger(), "Goal was canceled"); return BT::NodeStatus::FAILURE;
        default: RCLCPP_ERROR(node_->get_logger(), "Unknown result code"); return BT::NodeStatus::FAILURE;
        }

        if (_aborted) {
            // this happens only if method halt() was invoked
            //_client.cancelAllGoals();
            RCLCPP_INFO(node_->get_logger(), "Yac supply aborted");
            return BT::NodeStatus::FAILURE;
        }
        request = std::make_shared<yacyac_interface::srv::TTS::Request>();
        request->tts_str_t = "약 제조가 완료되었습니다.";
        result = TTS_client_->async_send_request(request);
        RCLCPP_INFO(node_->get_logger(), "Yac supply return received");
        return BT::NodeStatus::SUCCESS;
    }

private:
    bool _aborted;

    // auto node_ = std::make_shared<rclcpp::Node>("nav2_client");
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<yacyac_interface::srv::TTS>::SharedPtr TTS_client_;

    const std::string TTS_service_name = "/yacyac/io";
};