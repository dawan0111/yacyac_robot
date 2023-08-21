#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include "yacyac_core/message.hpp"
#include "yacyac_core/qr_client.hpp"

int main(int argc, char** argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    const std::string packagePath = ament_index_cpp::get_package_share_directory("yacyac_core");

    auto nh = rclcpp::Node::make_shared("yacyac_core");
    nh->declare_parameter("bt_xml", "default.xml");
    std::string bt_xml;
    nh->get_parameter("bt_xml", bt_xml);
    bt_xml = packagePath + "/bt_xml/" + bt_xml;
    RCLCPP_INFO(nh->get_logger(), "Loading XML : %s", bt_xml.c_str());

    // We use the BehaviorTreeFactory to register our custom nodes
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<QRClient>("QRClient");
    factory.registerNodeType<Message>("Message");

    auto tree = factory.createTreeFromFile(bt_xml);
    tree.tickWhileRunning();

    return 0;
}