#include "nav2_client.cpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

using namespace BT;

int main(int argc, char** argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    const std::string packagePath = ament_index_cpp::get_package_share_directory("yacyac_core");

    auto nh = rclcpp::Node::make_shared("yacyac_core");
    nh->declare_parameter("bt_xml", "bt_nav_mememan.xml");
    std::string bt_xml;
    nh->get_parameter("bt_xml", bt_xml);
    bt_xml = packagePath + "/bt_xml/" + bt_xml;
    RCLCPP_INFO(nh->get_logger(), "Loading XML : %s", bt_xml.c_str());
    // RCLCPP_INFO(nh->get_logger(), "Loading XML : %s", bt_xml.c_str());

    // std::cout << "start" << std::endl;
    // We use the BehaviorTreeFactory to register our custom nodes

    BT::BehaviorTreeFactory factory;

    // RCLCPP_INFO(nh->get_logger(), "Loading XML : %s", bt_xml.c_str());
    factory.registerNodeType<Nav2Client>("Nav2Client");

    // Trees are created at deployment-time (i.e. at run-time, but only once at
    // the beginning). The currently supported format is XML. IMPORTANT: when the
    // object "tree" goes out of scope, all the TreeNodes are destroyed
    auto tree = factory.createTreeFromFile(bt_xml);

    // Create a logger
    StdCoutLogger logger_cout(tree);

    NodeStatus status = NodeStatus::RUNNING;
    // Keep on ticking until you get either a SUCCESS or FAILURE state
    while (rclcpp::ok() && status == NodeStatus::RUNNING) {
        status = tree.tickWhileRunning();
        // Sleep 100 milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}