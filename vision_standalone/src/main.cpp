// Throwaway test harness: exercises pre_game's and line_projection's
// functions against a synthetic frame so one can see them run without a
// real camera or ROS.
#include "pre_game.h"
#include "line_projection.h"
#include "world_coordinates.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <cmath>

namespace
{
    // Stand-in for a real NAO camera frame until getting one: a plain
    // background with a green "bottle" ellipse offset by bottle_shift_x, so
    // one can exercise the pipeline without hardware. Detection is hue-based
    // now, so the bottle just needs a hue inside the green band (with some
    // real saturation) and the background needs to fall outside it - a
    // muted, low-saturation background keeps it out regardless of hue.
    cv::Mat make_test_frame(int bottle_shift_x)
    {
        cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(130, 125, 120));
        cv::ellipse(frame,
                    cv::Point(320 + bottle_shift_x, 300),
                    cv::Size(30, 90),
                    0, 0, 360,
                    cv::Scalar(34, 177, 76),
                    cv::FILLED);
        return frame;
    }
}

int main()
{
    const cv::Mat frame_a = make_test_frame(0);
    const cv::Mat frame_b = make_test_frame(15);

    const cv::Mat mask = pre_game::saturation_mask(frame_a);
    cv::imwrite("saturation_mask.png", mask);
    std::cout << "wrote saturation_mask.png\n";

    const pre_game::BottleState state = pre_game::bottle_detected(frame_a);
    std::cout << "bottle_detected -> " << static_cast<int>(state) << '\n';

    const bool moving = pre_game::movement_detected(frame_a, frame_b);
    std::cout << "movement_detected -> " << std::boolalpha << moving << '\n';

    const line_projection::PointingArea area = line_projection::find_pointing_area(frame_a);
    std::cout << "find_pointing_area -> direction=" << static_cast<int>(area.direction)
              << " top=" << area.top << " bottom=" << area.bottom
              << " left=" << area.left << " right=" << area.right << '\n';

    if (const auto line = line_projection::compute_pointing_line(frame_a, area))
    {
        std::cout << "compute_pointing_line -> angle=" << line->angle_degrees << " degrees\n";

        cv::Mat annotated = frame_a.clone();
        line_projection::draw_pointing_line(annotated, *line);
        cv::imwrite("pointing_line.png", annotated);
        std::cout << "wrote pointing_line.png\n";

        const world_coordinates::WorldPoint world_point =
            world_coordinates::to_world_coordinates(line->ellipse.center, line->angle_degrees, frame_a);
        std::cout << "to_world_coordinates -> x=" << world_point.x << " y=" << world_point.y
                  << " angle=" << world_point.angle_degrees << " degrees\n";

        if (const auto target = world_coordinates::find_target_point(world_point))
        {
            std::cout << "find_target_point -> (" << target->x << ", " << target->y << ")\n";

            const world_coordinates::HeadPose pose = world_coordinates::compute_head_pose(*target);
            std::cout << "compute_head_pose -> pitch=" << pose.pitch_radians
                      << " rad, yaw=" << pose.yaw_radians << " rad ("
                      << pose.yaw_radians * 180.0 / M_PI << " degrees)\n";
        }
        else
        {
            std::cout << "find_target_point -> nullopt (ask for the bottle to be spun again)\n";
        }
    }
    else
    {
        std::cout << "compute_pointing_line -> not enough points to fit an ellipse\n";
    }

    return 0;
}
