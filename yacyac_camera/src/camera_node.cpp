#include "yacyac_camera/camera_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<CameraNode> camera_node = nullptr;
    try {
        camera_node = std::make_shared<CameraNode>("/yacyac_top_camera", "yacyac_top_camera", "/dev/video1");
    }
    catch (const std::exception& e) {
        fprintf(stderr, "%s Exiting..\n", e.what());
        return 1;
    }

    rclcpp::spin(camera_node);

    rclcpp::shutdown();

    return 0;
}