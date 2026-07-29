#include "world_coordinates.h"

#include <algorithm>
#include <cmath>

namespace world_coordinates
{

    WorldPoint to_world_coordinates(const cv::Point2d& pixel_center, double angle_degrees, const cv::Mat& frame)
    {
        double kPixelsPerCm = 2.5;
        double kNaoDistanceFromBoard = 45;

        // Pixel coordinates have (0,0) at the top-left with y growing downward;
        // flip to a bottom-left origin to work in ordinary Cartesian coordinates.
        const double cartesian_y = frame.rows - pixel_center.y;

        // Fold the angle into this module's 0-360 convention.
        const double cartesian_angle = (angle_degrees < 0) ? -angle_degrees : 360.0 - angle_degrees;

        // Camera sensor is centered in the middle of the image.
        const double camera_x = pixel_center.x - frame.cols / 2.0;
        const double camera_y = cartesian_y - frame.rows / 2.0;

        WorldPoint point;
        point.x = camera_x / kPixelsPerCm;
        point.y = camera_y / kPixelsPerCm + kNaoDistanceFromBoard;
        point.angle_degrees = cartesian_angle;
        return point;
    }

    std::optional<cv::Point2d> find_target_point(const WorldPoint& point)
    {
        return {};
    }

    HeadPose compute_head_pose(const cv::Point2d& target_point)
    {
        return {};
    }

    HeadPose check_pose_correctness_agains_w_coordinates(const cv::Point2d& target_point);

}
