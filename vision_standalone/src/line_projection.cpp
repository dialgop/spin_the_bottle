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
    return;
}

}