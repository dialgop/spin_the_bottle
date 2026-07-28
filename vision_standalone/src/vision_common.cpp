#include "vision_common.h"

#include <opencv2/imgproc.hpp>

namespace
{
    constexpr int kDetectionThreshold = 70;
}

namespace vision_common
{

    cv::Mat threshold_mask(const cv::Mat& bgr_image)
    {
        return cv::Mat{};
    }

    std::vector<cv::Point> convex_hull_of(const cv::Mat& binary_mask)
    {
        return std::vector<cv::Point>{};
    }

}
