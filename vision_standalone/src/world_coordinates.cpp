#include "world_coordinates.h"

#include <algorithm>
#include <cmath>

namespace
{
    // NAO stands with its feet at the world-coordinate origin; the board
    // (and therefore the bottle) sits this many cm further along +y.
    constexpr double kNaoDistanceFromBoard = 45;

    // Calibrated: about 2.5 camera pixels per cm of physical distance.
    constexpr double kPixelsPerCm = 2.5;

    // Radius (cm) of the circle of players around the board - 1 meter.
    constexpr double kPlayerCircleRadius = 100;

    // Returns whichever of a/b has the smaller y, or std::nullopt if even
    // that smaller value is at or behind NAO (y <= 0).
    std::optional<cv::Point2d> pick_point_with_smaller_y(const cv::Point2d& a, const cv::Point2d& b)
    {
        const cv::Point2d& smaller = (a.y < b.y) ? a : b;
        if (smaller.y <= 0)
        {
            return std::nullopt;
        }
        return smaller;
    }
}

namespace world_coordinates
{

    WorldPoint to_world_coordinates(const cv::Point2d& pixel_center, double angle_degrees, const cv::Mat& frame)
    {
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
        const double angle = point.angle_degrees;

        // NAO can't resolve a target in this band to an actual player at the
        // table - ask for the bottle to be spun again.
        if (angle >= 205 && angle < 335)
        {
            return std::nullopt;
        }

        // Near-vertical pointing line (x is ~constant): the general slope-based
        // line equation below breaks down here since tan(90 degrees) is
        // effectively infinite, so this is solved directly instead.
        if (angle > 85 && angle < 95)
        {
            const double discriminant = kPlayerCircleRadius * kPlayerCircleRadius - point.x * point.x;
            if (discriminant < 0)
            {
                return std::nullopt;
            }
            return cv::Point2d(point.x, std::sqrt(discriminant));
        }

        return {};
    }

    HeadPose compute_head_pose(const cv::Point2d& target_point)
    {
        return {};
    }

    HeadPose check_pose_correctness_agains_w_coordinates(const cv::Point2d& target_point);

}
