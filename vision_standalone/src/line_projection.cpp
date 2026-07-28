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
    return {};
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