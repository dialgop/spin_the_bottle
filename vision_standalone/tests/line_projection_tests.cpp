#include "line_projection.h"

#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>

namespace
{
    int failure_count = 0;

    void check(bool condition, const char* description)
    {
        std::cout << (condition ? "PASS " : "FAIL ") << description << '\n';
        if (!condition) ++failure_count;
    }

    void check_near(const char* name, double actual, double expected, double tolerance)
    {
        const bool matches = std::abs(actual - expected) < tolerance;
        std::cout << (matches ? "PASS " : "FAIL ") << name << ": got " << actual
                  << ", expected " << expected << " +/- " << tolerance << '\n';
        if (!matches) ++failure_count;
    }

    cv::Mat make_bottle_shape_frame()
    {
        cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(130, 125, 120));

        cv::circle(frame, cv::Point(300, 280), 60, cv::Scalar(34, 177, 76), cv::FILLED);
        cv::rectangle(frame, cv::Point(290, 150), cv::Point(310, 210), cv::Scalar(34, 177, 76), cv::FILLED);

        return frame;
    }

    cv::Mat make_rotated_ellipse_frame(double rotation_degrees)
    {
        cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(130, 125, 120));
        cv::ellipse(frame, cv::Point(320, 260), cv::Size(20, 90), rotation_degrees, 0, 360,
                    cv::Scalar(34, 177, 76), cv::FILLED);
        return frame;
    }
}

int main()
{
    {
        const cv::Mat frame = make_bottle_shape_frame();
        const line_projection::PointingArea area = line_projection::find_pointing_area(frame);

        check(area.direction == line_projection::PointingDirection::Up,
              "find_pointing_area points toward the narrow neck (Up)");
        check_near("find_pointing_area top", area.top, 150, 5);
        check_near("find_pointing_area bottom", area.bottom, 340, 5);
        check_near("find_pointing_area left", area.left, 240, 5);
        check_near("find_pointing_area right", area.right, 360, 5);
    }
}
