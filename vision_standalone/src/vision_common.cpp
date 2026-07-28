#include "vision_common.h"

#include <opencv2/imgproc.hpp>

namespace
{
    // OpenCV packs the usual 0-360 degree hue wheel into a single byte
    // (0-179), so these are half of the "normal" hue degrees you'd read
    // off a color picker. This band covers yellow-green through cyan-green,
    // calibrated for a green bottle (wine or Sprite).
    constexpr int kBottleHueLow = 35;
    constexpr int kBottleHueHigh = 85;

    // Hue is only meaningful once a pixel actually has some color to it -
    // right at the gray/black/white axis (very low saturation), the hue
    // angle is numerically unstable. This floor just excludes those
    // pixels; it's deliberately loose, since the whole point of matching
    // on hue is to keep working even when lighting pushes saturation down.
    constexpr int kMinSaturationForHue = 40;
}

namespace vision_common
{

    cv::Mat threshold_mask(const cv::Mat& bgr_image)
    {
        cv::Mat hsv_image;
        cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

        cv::Mat channels[3];
        cv::split(hsv_image, channels);

        // Which pixels fall in the bottle's known hue band - this is the
        // part that keeps working under lighting changes that would wash
        // out a saturation/value-only check.
        cv::Mat hue_mask;
        cv::inRange(channels[0], kBottleHueLow, kBottleHueHigh, hue_mask);

        cv::Mat saturation_floor;
        cv::threshold(channels[1], saturation_floor, kMinSaturationForHue, 255, cv::THRESH_BINARY);

        cv::Mat mask;
        cv::bitwise_and(hue_mask, saturation_floor, mask);

        // Blank out everything except the central band where the bottle can
        // be: rows below the top quarter, columns within the middle two
        // thirds - keeps contours from the board/table edges out of it.
        for (int row = 0; row < mask.rows; ++row)
        {
            for (int col = 0; col < mask.cols; ++col)
            {
                const bool inside_band = row > mask.rows / 4 && col > mask.cols / 6 && col < mask.cols * 5 / 6;

                if (!inside_band)
                {
                    mask.at<uchar>(row, col) = 0;
                }
            }
        }

        return mask;
    }

    std::vector<cv::Point> convex_hull_of(const cv::Mat& binary_mask)
    {
        return std::vector<cv::Point>{};
    }

}
