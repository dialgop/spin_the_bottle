#pragma once

#include <opencv2/core.hpp>
#include <vector>

// Small pieces shared by pre_game and line_projection so the same
// "find the bottle-colored blob" logic isn't duplicated between them.
namespace vision_common
{
    // Same raw black/white mask that pre_game::saturation_mask builds on top
    // of (HSV saturation/value threshold, cropped to the central band), but
    // WITHOUT filling in the convex hull. Some callers need the unfilled
    // version (e.g. to fit an ellipse to just the outline).
    cv::Mat threshold_mask(const cv::Mat& bgr_image);

    std::vector<cv::Point> convex_hull_of(const cv::Mat& binary_mask);
}
