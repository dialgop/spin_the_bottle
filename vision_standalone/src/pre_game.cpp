#include "pre_game.h"
#include "vision_common.h"

#include <opencv2/imgproc.hpp>
#include <iostream>

namespace
{
    constexpr int kBottleThreshold = 200;
    constexpr int kBottleIncomplete = 20;
}

namespace pre_game
{

cv::Mat saturation_mask(const cv::Mat& bgr_image)
{
    cv::Mat mask = vision_common::threshold_mask(bgr_image);

    const std::vector<cv::Point> hull = vision_common::convex_hull_of(mask);
    if (!hull.empty())
    {
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