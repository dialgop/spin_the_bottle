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

    // height (cm) of Nao camera
    constexpr double kNaoBottomCameraHeight = 27.5;

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

        // General case: intersect the line through (point.x, point.y) with
        // slope tan(angle) against the circle of radius kPlayerCircleRadius
        // centered on NAO. Substituting y = point.y + slope*(x - point.x) into
        // x^2 + y^2 = r^2 and expanding gives:
        const double angle_radians = angle * M_PI / 180.0;
        const double slope = std::tan(angle_radians);
        const double intercept = point.y - slope * point.x; // where the line crosses x=0

        const double a = 1 + slope * slope;
        const double b = 2 * slope * intercept;
        const double c = intercept * intercept - kPlayerCircleRadius * kPlayerCircleRadius;

        const double discriminant = b * b - 4 * a * c;
        if (discriminant < 0)
        {
            return std::nullopt;
        }

        const double sqrt_discriminant = std::sqrt(discriminant);
        const double x1 = (-b + sqrt_discriminant) / (2 * a);
        const double x2 = (-b - sqrt_discriminant) / (2 * a);
        const double y1 = point.y + slope * (x1 - point.x);
        const double y2 = point.y + slope * (x2 - point.x);

        if ((angle >= 0 && angle < 85) || (angle > 95 && angle < 180))
        {
            // Two candidate roots on opposite sides of the bottle; pick the one
            // on the side the angle actually points towards.
            if (angle < 90)
            {
                return (x1 < 0) ? cv::Point2d(x2, y2) : cv::Point2d(x1, y1);
            }
            return (x1 < 0) ? cv::Point2d(x1, y1) : cv::Point2d(x2, y2);
        }

        // Remaining bands (180-205, 335-360): pick whichever root is nearer to
        // NAO, and only accept it if it's actually in front (y > 0).
        return pick_point_with_smaller_y(cv::Point2d(x1, y1), cv::Point2d(x2, y2));

    }

    HeadPose compute_head_pose(const cv::Point2d& target_point)
    {
        static const double kPitch = std::atan2(kNaoBottomCameraHeight, kPlayerCircleRadius);

        const double yaw_degrees = std::atan2(target_point.y, target_point.x) * 180.0 / M_PI;

        // Snap into one of 5 fixed head positions, 36 degrees apart, spanning
        // the 0-180 degree forward arc.
        double snapped_degrees = yaw_degrees;
        if (yaw_degrees >= 0 && yaw_degrees < 36) snapped_degrees = 18;
        else if (yaw_degrees >= 36 && yaw_degrees < 72) snapped_degrees = 54;
        else if (yaw_degrees >= 72 && yaw_degrees < 108) snapped_degrees = 90;
        else if (yaw_degrees >= 108 && yaw_degrees < 144) snapped_degrees = 126;
        else if (yaw_degrees >= 144 && yaw_degrees <= 180) snapped_degrees = 162;

        // find_target_point should only ever hand back points in front of NAO
        // (y > 0), which keeps yaw_degrees within [0, 180]. Clamp anyway before
        // this becomes a physical head movement - NAO's neck should never be
        // asked to turn further than its actual joint limits allow.
        snapped_degrees = std::clamp(snapped_degrees, 0.0, 180.0);

        // Shift so 90 degrees (straight ahead) becomes 0 - what NAO's HeadYaw
        // joint actually expects.
        const double yaw_radians = (snapped_degrees - 90.0) * M_PI / 180.0;

        return HeadPose{kPitch, yaw_radians};

    }

}
