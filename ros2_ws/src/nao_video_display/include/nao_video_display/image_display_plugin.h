#pragma once

#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core.hpp>

#include <mutex>
#include <optional>

// Custom webots_ros2_driver plugin: renders the same recorded bottle-spin
// footage referee_node is watching onto a Webots Display device, so it's
// visible inside the running simulation, not just in the terminal log.
// webots_ros2_driver has no built-in support for Display (unlike Camera or
// the ros2_control joints), so this is a from-scratch plugin rather than
// a <device> declaration in URDF.
namespace nao_video_display
{
    class ImageDisplayPlugin : public webots_ros2_driver::PluginInterface
    {
    public:
        void init(webots_ros2_driver::WebotsNode* node, std::unordered_map<std::string, std::string>& parameters) override;
        void step() override;

    private:
        void on_image(const sensor_msgs::msg::Image::SharedPtr message);

        webots_ros2_driver::WebotsNode* node_ = nullptr;
        int display_tag_ = 0;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

        // The subscription callback and step() may run on different
        // executor threads - the pending frame is handed off through here
        // rather than drawn directly from the callback.
        std::mutex mutex_;
        std::optional<cv::Mat> pending_frame_;
    };
}
