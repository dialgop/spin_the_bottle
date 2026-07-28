#pragma once

#include <opencv2/core.hpp>
#include <vector>

// Small pieces shared by pre_game and line_projection so the same
// "find the bottle-colored blob" logic isn't duplicated between them.
namespace vision_common
{
    cv::Mat threshold_mask(const cv::Mat& bgr_image);
    std::vector<cv::Point> convex_hull_of(const cv::Mat& binary_mask);
}
