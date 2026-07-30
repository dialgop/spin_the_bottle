#pragma once

#include <opencv2/core.hpp>
#include <optional>

namespace world_coordinates
{
    struct WorldPoint
    {
        double x;             // cm, relative to NAO's feet
        double y;             // cm, relative to NAO's feet ("forward" is positive)
        double angle_degrees; // bottle's pointing angle, in this module's 0-360 convention
    };

    WorldPoint to_world_coordinates(const cv::Point2d& pixel_center, double angle_degrees, const cv::Mat& frame);

    std::optional<cv::Point2d> find_target_point(const WorldPoint& point);

    struct HeadPose
    {
        double pitch_radians;
        double yaw_radians; // 0 = straight ahead - ready to send to NAO's HeadYaw joint directly
    };

    HeadPose compute_head_pose(const cv::Point2d& target_point);

}
