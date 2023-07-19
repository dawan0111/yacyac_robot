#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "yacyac_camera/camera_node.hpp"
#include "yacyac_camera/qr_detector_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    auto param_node = rclcpp::Node("param_node");

    param_node.declare_parameter("top_camera.node_name", "yacyac_top_camera");
    param_node.declare_parameter("top_camera.device", "/dev/video0");
    param_node.declare_parameter("top_camera.topic_name", "/yacyac_top_camera");
    param_node.declare_parameter("top_camera.frame_width", 640);
    param_node.declare_parameter("top_camera.frame_height", 480);
    param_node.declare_parameter("qr.node_name", "yacyac_qr_detector");
    param_node.declare_parameter("qr.camera_topic_name", "/yacyac_top_camera");
    param_node.declare_parameter("qr.topic_name", "/qr_node");
    std::shared_ptr<CameraNode> camera_node = nullptr;
    try {
        camera_node = std::make_shared<CameraNode>(param_node.get_parameter("top_camera.topic_name").as_string(), param_node.get_parameter("top_camera.node_name").as_string(),
                                                   param_node.get_parameter("top_camera.device").as_string(), param_node.get_parameter("top_camera.frame_width").as_int(),
                                                   param_node.get_parameter("top_camera.frame_height").as_int());
    }
    catch (const std::exception& e) {
        fprintf(stderr, "%s Exiting..\n", e.what());
        return 1;
    }
    auto qr_detector_node = std::make_shared<QrDetectorNode>(param_node.get_parameter("qr.node_name").as_string(), param_node.get_parameter("qr.camera_topic_name").as_string(),
                                                             param_node.get_parameter("qr.topic_name").as_string());

    executor.add_node(camera_node);
    executor.add_node(qr_detector_node);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}