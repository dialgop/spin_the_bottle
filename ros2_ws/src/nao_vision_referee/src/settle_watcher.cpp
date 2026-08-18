#include "settle_watcher.h"

#include "face_detection.h"
#include "line_projection.h"
#include "pre_game.h"

#include <opencv2/videoio.hpp>

#include <chrono>
#include <thread>

namespace settle_watcher
{
    std::optional<world_coordinates::HeadPose> find_settled_head_pose(const std::string& video_path,
                                                                       const rclcpp::Logger& logger)
    {
        constexpr int kSettleStreak = 5;

        cv::VideoCapture capture(video_path);
        if (!capture.isOpened())
        {
            RCLCPP_ERROR(logger, "failed to open video: %s", video_path.c_str());
            return std::nullopt;
        }

        const double fps = capture.get(cv::CAP_PROP_FPS) > 0.0 ? capture.get(cv::CAP_PROP_FPS) : 30.0;
        const auto frame_period = std::chrono::duration<double>(1.0 / fps);

        cv::Mat previous_frame, frame;
        bool seen_movement = false;
        int still_streak = 0;
        int frame_index = 0;
        while (capture.read(frame))
        {
            std::this_thread::sleep_for(frame_period);

            if (!previous_frame.empty())
            {
                const bool moving = pre_game::movement_detected(frame, previous_frame);
                if (moving)
                {
                    seen_movement = true;
                    still_streak = 0;
                }
                else if (seen_movement)
                {
                    ++still_streak;
                    if (still_streak >= kSettleStreak)
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
                }
            }
            previous_frame = frame.clone();
            ++frame_index;
        }

        RCLCPP_WARN(logger, "bottle never settled in %d frame(s) from %s", frame_index, video_path.c_str());
        return std::nullopt;
    }

    bool video_has_face(const std::string& video_path, const rclcpp::Logger& logger)
    {
        cv::VideoCapture capture(video_path);
        if (!capture.isOpened())
        {
            RCLCPP_ERROR(logger, "failed to open video: %s", video_path.c_str());
            return false;
        }

        cv::Mat frame;
        while (capture.read(frame))
        {
            if (face_detection::detect_face(frame))
            {
                return true;
            }
        }
        return false;
    }
}
