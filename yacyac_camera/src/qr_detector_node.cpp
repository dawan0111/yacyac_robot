#include "yacyac_camera/qr_detector_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<QrDetectorNode> qr_detector_node = nullptr;
    try {
        qr_detector_node =
            std::make_shared<QrDetectorNode>("/qr_code", "/yacyac_camera", "qr_detector_node");
    }
    catch (const std::exception& e) {
        fprintf(stderr, "%s Exiting..\n", e.what());
        return 1;
    }

    rclcpp::spin(qr_detector_node);

    rclcpp::shutdown();

    return 0;
}