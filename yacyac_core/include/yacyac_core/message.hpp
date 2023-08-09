#ifndef MESSAGE_HPP_
#define MESSAGE_HPP_

#include <behaviortree_cpp/behavior_tree.h>

class Message : public BT::SyncActionNode {
public:
    Message(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config) {}
    ~Message() {};

    static BT::PortsList providedPorts()
    {
        // This action has a single input port called "message"
        return { BT::InputPort<std::string>("message") };
    }

    BT::NodeStatus tick() override;
private:
};

#endif
