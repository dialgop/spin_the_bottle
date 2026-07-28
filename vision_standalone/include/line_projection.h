#pragma once

#include <opencv2/core.hpp>

namespace line_projection
{

    enum class PointingDirection
    {
        Left,
        Up,
        Right,
        Down
    };

    struct PointingArea
    {
        PointingDirection direction;
        int top;
        int bottom;
        int left;
        int right;
    };

    struct PointingLine
    {
        cv::RotatedRect ellipse;
        double angle_degrees;
        cv::Point2d line_end;
    };
}