#pragma once

#include <opencv2/core.hpp>
#include <optional>
#include <string>

namespace bottle_detector
{
    struct DetectedBottle
    {
        cv::Rect bounds;  // bounding box, in frame pixel coordinates
        cv::Mat mask;     // binary mask (0/255), same size as the input frame
        float confidence;
    };

    bool load_model(const std::string& model_path);

    std::optional<DetectedBottle> detect_bottle(const cv::Mat& bgr_image);
}
