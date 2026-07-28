#include "line_projection.h"
#include "vision_common.h"

#include <opencv2/imgproc.hpp>
#include <cmath>

namespace line_projection
{

PointingArea find_pointing_area(const cv::Mat& bgr_image)
{
    cv::Mat mask = vision_common::threshold_mask(bgr_image);
    const std::vector<cv::Point> hull = vision_common::convex_hull_of(mask);

    // find_pointing_area is only meaningful once pre_game has confirmed a
    // bottle is actually in frame; an empty hull here means that precondition
    // was skipped, so fall back to a degenerate all-zero area rather than
    // crashing on boundingRect/countNonZero of nothing.
    if (hull.empty())
    {
        return PointingArea{PointingDirection::Up, 0, 0, 0, 0};
    }

    cv::fillConvexPoly(mask, hull, cv::Scalar(255));
    const cv::Rect bounds = cv::boundingRect(hull);

    const int top = bounds.y;
    const int bottom = bounds.y + bounds.height;
    const int left = bounds.x;
    const int right = bounds.x + bounds.width;

    PointingDirection direction;

    if ((bottom - top) >= (right - left))
    {
        const int half_height = (bottom - top) / 2;
        const cv::Rect upper_half(left, top, right - left, half_height);
        const cv::Rect lower_half(left, top + half_height, right - left, (bottom - top) - half_height);

        const int pix_up = cv::countNonZero(mask(upper_half));
        const int pix_down = cv::countNonZero(mask(lower_half));

        direction = (pix_up < pix_down) ? PointingDirection::Up : PointingDirection::Down;
    }
    else
    {
        const int half_width = (right - left) / 2;
        const cv::Rect left_half(left, top, half_width, bottom - top);
        const cv::Rect right_half(left + half_width, top, (right - left) - half_width, bottom - top);

        const int pix_left = cv::countNonZero(mask(left_half));
        const int pix_right = cv::countNonZero(mask(right_half));

        direction = (pix_left < pix_right) ? PointingDirection::Left : PointingDirection::Right;
    }

    return PointingArea{direction, top, bottom, left, right};
}

std::optional<PointingLine> compute_pointing_line(const cv::Mat& bgr_image, const PointingArea& area)
{
    const cv::Mat mask = vision_common::threshold_mask(bgr_image);
    const std::vector<cv::Point> hull = vision_common::convex_hull_of(mask);

    // fitEllipse requires at least 5 points to fit an ellipse through.
    if (hull.size() < 5)
    {
        return std::nullopt;
    }

    const cv::RotatedRect min_ellipse = cv::fitEllipse(hull);
    double angle = min_ellipse.angle - 90;

    // An ellipse fit has no notion of "front" or "back" - it's symmetric.
    // area.direction (found separately, from which half of the bounding box
    // had fewer pixels) tells us which of the two possible pointing angles
    // is actually correct, so flip by 180 degrees when the raw angle picked
    // the wrong one.
    switch (area.direction)
    {
        case PointingDirection::Left:
            if ((angle > -90 && angle < 0) || (angle > 0 && angle < 90))
            {
                angle += 180;
            }
            break;
        case PointingDirection::Up:
            if (angle > 0 && angle < 180)
            {
                angle += 180;
            }
            break;
        case PointingDirection::Right:
            if (angle > 90 && angle < 270)
            {
                angle += 180;
            }
            break;
        case PointingDirection::Down:
            if ((angle > -90 && angle < 0) || (angle > 180 && angle < 270))
            {
                angle += 180;
            }
            break;
    }

    PointingLine line;
    line.ellipse = min_ellipse;
    line.angle_degrees = angle;
    line.line_end.x = min_ellipse.center.x + 100 * std::cos(angle * M_PI / 180);
    line.line_end.y = min_ellipse.center.y + 100 * std::sin(angle * M_PI / 180);

    return line;
}

    void draw_pointing_line(cv::Mat& image, const PointingLine& line)
    {
        cv::ellipse(image, line.ellipse, cv::Scalar(180), 2, 8);

        const cv::Point center(cv::saturate_cast<int>(line.ellipse.center.x), cv::saturate_cast<int>(line.ellipse.center.y));
        const cv::Point endpoint(cv::saturate_cast<int>(line.line_end.x), cv::saturate_cast<int>(line.line_end.y));
        const double angle_rad = line.angle_degrees * M_PI / 180;

        cv::line(image, center, endpoint, cv::Scalar(200), 4);

        // Two short "hook" lines near the tip, angled +/-45 degrees off the main
        // line, to draw an arrowhead shape.
        const cv::Point hook1(
            cv::saturate_cast<int>(endpoint.x - 12 * std::cos(angle_rad + M_PI / 4)),
            cv::saturate_cast<int>(endpoint.y - 12 * std::sin(angle_rad + M_PI / 4)));
        cv::line(image, hook1, endpoint, cv::Scalar(200), 4);

        const cv::Point hook2(
            cv::saturate_cast<int>(endpoint.x - 12 * std::cos(angle_rad - M_PI / 4)),
            cv::saturate_cast<int>(endpoint.y - 12 * std::sin(angle_rad - M_PI / 4)));
        cv::line(image, hook2, endpoint, cv::Scalar(200), 4);
    }


}