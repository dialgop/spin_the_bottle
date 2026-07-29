#include "world_coordinates.h"

#include <algorithm>
#include <cmath>

namespace world_coordinates
{

    WorldPoint to_world_coordinates(const cv::Point2d& pixel_center, double angle_degrees, const cv::Mat& frame)
    {
        return {};
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
