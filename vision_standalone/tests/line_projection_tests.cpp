// Correctness checks for line_projection against synthetic shapes with
// known geometry. Unlike world_coordinates' quadratic, fitEllipse's output
// isn't something derivable by hand, so these checks are looser tolerance
// "does this land near what the known geometry implies" rather than exact
// hand-derived values. Exits non-zero if any check fails, so this can plug
// into ctest.
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

    // Builds a synthetic "bottle-like" silhouette: a wide green body plus a
    // narrow green neck sticking out above it, on a low-saturation
    // background - deliberately asymmetric (unlike a plain ellipse) so
    // find_pointing_area's "thinner side has fewer pixels" logic has an
    // actual answer to find. Kept below row 120 (frame.rows/4) so none of
    // it falls inside threshold_mask's cropped-out top band.
    cv::Mat make_bottle_shape_frame()
    {
        cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(130, 125, 120));

        cv::circle(frame, cv::Point(300, 280), 60, cv::Scalar(34, 177, 76), cv::FILLED);
        cv::rectangle(frame, cv::Point(290, 150), cv::Point(310, 210), cv::Scalar(34, 177, 76), cv::FILLED);

        return frame;
    }

    // Builds a synthetic elongated green ellipse rotated by rotation_degrees
    // (OpenCV's ellipse-drawing angle convention), on a low-saturation
    // background, so compute_pointing_line has a known axis to recover.
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
    // find_pointing_area: the neck sticks out above the body (smaller row
    // values = "up" in image space) and is much thinner than the body, so
    // the top half of the bounding box should have far fewer pixels.
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

    // compute_pointing_line: an ellipse drawn rotated by a known angle
    // should be fit back to approximately that axis. area.direction is
    // supplied directly (rather than via find_pointing_area) to isolate
    // this from the direction-detection logic tested above.
    {
        const cv::Mat frame = make_rotated_ellipse_frame(30);
        const line_projection::PointingArea area{line_projection::PointingDirection::Up, 0, 0, 0, 0};

        if (const auto line = line_projection::compute_pointing_line(frame, area))
        {
            // fitEllipse's returned angle uses the same convention as
            // cv::ellipse's drawing angle, so it recovers ~30 here (verified
            // empirically: 30.03). compute_pointing_line then subtracts 90
            // (angle_degrees = ellipse.angle - 90), and PointingDirection::Up
            // doesn't trigger the +180 correction for a result already
            // outside (0,180) - so the expected result is 30 - 90 = -60.
            check_near("compute_pointing_line recovers rotated ellipse angle",
                       line->angle_degrees, -60, 2);
        }
        else
        {
            check(false, "compute_pointing_line found a line for the rotated ellipse");
        }
    }

    // draw_pointing_line: pure side effect - just confirm it actually
    // changes the image rather than being a no-op or crashing.
    {
        const cv::Mat frame = make_rotated_ellipse_frame(30);
        const line_projection::PointingArea area = line_projection::find_pointing_area(frame);

        if (const auto line = line_projection::compute_pointing_line(frame, area))
        {
            cv::Mat annotated = frame.clone();
            line_projection::draw_pointing_line(annotated, *line);

            cv::Mat diff;
            cv::absdiff(frame, annotated, diff);
            check(cv::countNonZero(diff.reshape(1)) > 0, "draw_pointing_line actually draws something");
        }
        else
        {
            check(false, "compute_pointing_line found a line to draw");
        }
    }

    if (failure_count > 0)
    {
        std::cout << failure_count << " check(s) FAILED\n";
        return 1;
    }

    std::cout << "all checks passed\n";
    return 0;
}
