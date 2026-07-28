#include "line_projection.h"
#include "vision_common.h"

#include <opencv2/imgproc.hpp>
#include <cmath>

namespace line_projection
{

PointingArea find_pointing_area(const cv::Mat& bgr_image)
{
    return {};
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