#include "nao_video_display/image_display_plugin.h"

#include <cv_bridge/cv_bridge.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <webots/display.h>
#include <webots/robot.h>

#include <opencv2/imgproc.hpp>

namespace
{
    constexpr const char* kDisplayName = "screen";
    constexpr const char* kImageTopic = "/bottle_video/image";
}

namespace nao_video_display
{
    void ImageDisplayPlugin::init(webots_ros2_driver::WebotsNode* node, std::unordered_map<std::string, std::string>& parameters)
    {
        (void)parameters;
        node_ = node;

        display_tag_ = wb_robot_get_device(kDisplayName);
        if (display_tag_ == 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "nao_video_display: no Display device named '%s'", kDisplayName);
        }

        subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
            kImageTopic, 10, std::bind(&ImageDisplayPlugin::on_image, this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(), "nao_video_display: rendering %s onto display '%s'", kImageTopic, kDisplayName);
    }

    void ImageDisplayPlugin::on_image(const sensor_msgs::msg::Image::SharedPtr message)
    {
        try
        {
            const auto cv_image = cv_bridge::toCvCopy(message, "bgr8");
            std::lock_guard<std::mutex> lock(mutex_);
            pending_frame_ = cv_image->image;
        }
        catch (const cv_bridge::Exception& error)
        {
            RCLCPP_ERROR(node_->get_logger(), "nao_video_display: cv_bridge error: %s", error.what());
        }
    }

    void ImageDisplayPlugin::step()
    {
        if (display_tag_ == 0) return;

        std::optional<cv::Mat> frame;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            frame = pending_frame_;
            pending_frame_.reset();
        }
        if (!frame) return;

        const int width = wb_display_get_width(display_tag_);
        const int height = wb_display_get_height(display_tag_);

        cv::Mat resized;
        cv::resize(*frame, resized, cv::Size(width, height));
        cv::Mat rgb;
        cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

        WbImageRef image = wb_display_image_new(display_tag_, width, height, rgb.data, WB_IMAGE_RGB);
        wb_display_image_paste(display_tag_, image, 0, 0, false);
        wb_display_image_delete(display_tag_, image);
    }
}

PLUGINLIB_EXPORT_CLASS(nao_video_display::ImageDisplayPlugin, webots_ros2_driver::PluginInterface)
