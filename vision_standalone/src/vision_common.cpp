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
        cv::Mat hsv_image;
        cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

        cv::Mat channels[3];
        cv::split(hsv_image, channels);

        cv::Mat inverted_saturation;
        cv::bitwise_not(channels[1], inverted_saturation);
        for (int row = 0; row < inverted_saturation.rows; ++row)
        {
            for (int col = 0; col < inverted_saturation.cols; ++col)
            {
                const bool inside_band = row > inverted_saturation.rows / 4 && col > inverted_saturation.cols / 6 && col < inverted_saturation.cols * 5 / 6;

                if (!inside_band)
                {
                    inverted_saturation.at<uchar>(row, col) = 255;
                }
            }
        }

        cv::Mat saturation_threshold;
        cv::threshold(inverted_saturation, saturation_threshold, kDetectionThreshold, 255, cv::THRESH_BINARY_INV);

        cv::Mat value_threshold;
        cv::threshold(channels[2], value_threshold, kDetectionThreshold, 255, cv::THRESH_BINARY_INV);

        return saturation_threshold.mul(value_threshold);
    }

    std::vector<cv::Point> convex_hull_of(const cv::Mat& binary_mask)
    {
        return std::vector<cv::Point>{};
    }

}
