#include "pre_game.h"

#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>

namespace
{
    constexpr int kDetectionThreshold = 70;
    constexpr int kBottleThreshold = 200;
    constexpr int kBottleIncomplete = 20;
}

namespace pre_game
{

cv::Mat saturation_mask(const cv::Mat& bgr_image)
{
    cv::Mat hsv_image;
    cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

    cv::Mat channels[3];
    cv::split(hsv_image, channels);

    cv::Mat inverted_saturation;
    cv::bitwise_not(channels[1], inverted_saturation);

    // Blank out everything except the central band where the bottle can be:
    // rows below the top quarter, columns within the middle two thirds.
    for (int row = 0; row < inverted_saturation.rows; ++row)
    {
        for (int col = 0; col < inverted_saturation.cols; ++col)
        {
            const bool inside_band =
                row > inverted_saturation.rows / 4 &&
                col > inverted_saturation.cols / 6 &&
                col < inverted_saturation.cols * 5 / 6;

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

    cv::Mat mask = saturation_threshold.mul(value_threshold);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> merged_points;
    for (const auto& contour : contours)
    {
        merged_points.insert(merged_points.end(), contour.begin(), contour.end());
    }

    // Only fill the hull if something was actually found - convexHull on an
    // empty point set has nothing well-defined to do with it.
    if (!merged_points.empty())
    {
        std::vector<cv::Point> hull;
        cv::convexHull(merged_points, hull);
        cv::fillConvexPoly(mask, hull, cv::Scalar(255));
    }

    return mask;
}

BottleState bottle_detected(const cv::Mat& bgr_image)
{
    const cv::Mat mask = saturation_mask(bgr_image);
    const int white_pixel_count = cv::countNonZero(mask);
    const int frame_area = mask.rows * mask.cols;

    std::cout << "bottle_detected: white pixel count = " << white_pixel_count << '\n';

    if (white_pixel_count == frame_area)
    {
        std::cout << "starting the game, please wait for a moment\n";
        return BottleState::FullFrame;
    }
    if (white_pixel_count > kBottleThreshold)
    {
        return BottleState::Centered;
    }
    if (white_pixel_count > kBottleIncomplete)
    {
        std::cout << "please move the bottle more to the center\n";
        return BottleState::OffCenter;
    }

    std::cout << "it does not look like there is a bottle here\n";
    return BottleState::NotFound;
}

bool movement_detected(const cv::Mat& previous_frame, const cv::Mat& current_frame)
{
    const cv::Mat previous_mask = saturation_mask(previous_frame);
    const cv::Mat current_mask = saturation_mask(current_frame);

    cv::Mat difference;
    cv::absdiff(previous_mask, current_mask, difference);
    const int changed_pixel_count = cv::countNonZero(difference);
    const int previous_pixel_count = cv::countNonZero(previous_mask);

    std::cout << "changed pixels: " << changed_pixel_count
              << ", previous mask pixels: " << previous_pixel_count << '\n';

    const bool moving = previous_pixel_count > 0 && changed_pixel_count > previous_pixel_count / 3;
    std::cout << (moving ? "the bottle seems to be moving\n" : "the bottle is not moving\n");
    return moving;
}

}