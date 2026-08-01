#pragma once

#include <opencv2/core.hpp>
#include <optional>
#include <string>

namespace face_detection
{
    struct DetectedFace
    {
        cv::Rect bounds; // in frame pixel coordinates
    };

    bool load_cascade(const std::string& cascade_path);

    std::optional<DetectedFace> detect_face(const cv::Mat& bgr_image);
}
