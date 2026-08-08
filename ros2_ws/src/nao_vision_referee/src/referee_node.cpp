#include "line_projection.h"
#include "pre_game.h"
#include "world_coordinates.h"

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <opencv2/videoio.hpp>

#include <chrono>
#include <optional>

using namespace std::chrono_literals;

namespace
{
    std::optional<world_coordinates::HeadPose> find_settled_head_pose(std::string& video_path,
                                                                       rclcpp::Logger& logger)
    {
        cv::VideoCapture capture(video_path);
        if (!capture.isOpened())
        {
            RCLCPP_ERROR(logger, "failed to open video: %s", video_path.c_str());
            return std::nullopt;
        }

        cv::Mat previous_frame, frame;
        bool was_moving = false;
        int frame_index = 0;
        while (capture.read(frame))
        {
            if (!previous_frame.empty())
            {
                const bool moving = pre_game::movement_detected(frame, previous_frame);
                if (was_moving && !moving)
                {
                    RCLCPP_INFO(logger, "bottle settled at frame %d", frame_index);

                    const line_projection::PointingArea area = line_projection::find_pointing_area(frame);
                    const auto line = line_projection::compute_pointing_line(frame, area);
                    if (!line)
                    {
                        RCLCPP_WARN(logger, "settled frame has no fittable bottle silhouette");
                        return std::nullopt;
                    }

                    const world_coordinates::WorldPoint world_point =
                        world_coordinates::to_world_coordinates(line->ellipse.center, line->angle_degrees, frame);
                    const auto target = world_coordinates::find_target_point(world_point);
                    if (!target)
                    {
                        RCLCPP_WARN(logger, "pointing angle doesn't land on a valid target region");
                        return std::nullopt;
                    }

                    return world_coordinates::compute_head_pose(*target);
                }
                was_moving = moving;
            }
            previous_frame = frame.clone();
            ++frame_index;
        }

        RCLCPP_WARN(logger, "bottle never settled in %d frame(s) from %s", frame_index, video_path.c_str());
        return std::nullopt;
    }
}
