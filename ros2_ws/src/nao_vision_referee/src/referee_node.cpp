// Decides where NAO should point its head: runs vision_standalone's
// bottle-pointing pipeline against a recorded video, and once the bottle
// settles, publishes the resulting head pose as a JointTrajectory for
// nao_webots_driver's head_controller to execute.
//
// After publishing the pose, also checks a recorded face video for a face,
// the same way vision_standalone's face_detection_tests do - a stand-in for
// what NAO's camera would see after actually turning its head there, since
// there's no live/simulated camera yet (see ros2_ws/src/nao_webots_driver
// for why). This is only to test the wiring end-to-end; a real or simulated
// camera feed is the natural follow-up.
#include "face_detection.h"
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
    struct ArmPose
    {
        bool use_left_arm;
        double shoulder_pitch_radians;
        double shoulder_roll_radians;
    };

    ArmPose compute_arm_pose(double head_yaw_radians)
    {
        constexpr double kShoulderPitch = -0.3;  // raises the arm from resting (0 = horizontal forward)
        constexpr double kMaxShoulderRoll = 1.0; // stays inside NAO's real ~1.33 rad shoulder-roll range

        const bool use_left_arm = head_yaw_radians >= 0.0;
        const double roll_magnitude = std::min(std::abs(head_yaw_radians), kMaxShoulderRoll);
        const double shoulder_roll = use_left_arm ? roll_magnitude : -roll_magnitude;

        return ArmPose{use_left_arm, kShoulderPitch, shoulder_roll};
    }
    // Steps through video_path frame by frame until the bottle transitions
    // from moving to settled (mirrors vision_standalone/src/main.cpp's
    // per-frame pipeline), then returns the head pose that settle frame
    // points to. Returns std::nullopt if the video can't be opened, the
    // bottle never settles, or the settled frame doesn't yield a usable
    // target - callers only need to know whether there's somewhere to look.
    std::optional<world_coordinates::HeadPose> find_settled_head_pose(const std::string& video_path,
                                                                       const rclcpp::Logger& logger)
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

    // Scans every frame of video_path for a face, stopping at the first hit.
    // Mirrors face_detection_tests.cpp's any_frame_has_face helper.
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

class RefereeNode : public rclcpp::Node
{
public:
    RefereeNode() : rclcpp::Node("referee_node")
    {
        declare_parameter<std::string>("bottle_video_path", DEFAULT_BOTTLE_VIDEO_PATH);
        declare_parameter<std::string>("face_video_path", DEFAULT_FACE_VIDEO_PATH);

        head_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/head_controller/joint_trajectory", 10);
        arm_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);

        // Give head_controller/arm_controller a moment to come up before publishing.
        timer_ = create_wall_timer(2s, std::bind(&RefereeNode::run_once, this));
    }

private:
    void run_once()
    {
        timer_->cancel();

        const std::string video_path = get_parameter("bottle_video_path").as_string();
        const auto pose = find_settled_head_pose(video_path, get_logger());
        if (!pose)
        {
            RCLCPP_WARN(get_logger(), "no head pose to publish - ask for the bottle to be spun again");
            return;
        }

        trajectory_msgs::msg::JointTrajectory message;
        message.joint_names = {"HeadYaw", "HeadPitch"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {pose->yaw_radians, pose->pitch_radians};
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);
        message.points.push_back(point);

        RCLCPP_INFO(get_logger(), "publishing head pose: yaw=%f pitch=%f", pose->yaw_radians, pose->pitch_radians);
        publisher_->publish(message);

        const std::string face_video_path = get_parameter("face_video_path").as_string();
        if (video_has_face(face_video_path, get_logger()))
        {
            RCLCPP_INFO(get_logger(), "detect_face -> found someone - ask them to spin next");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "detect_face -> no one there - ask the group to spin again");
        }
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (!face_detection::load_model(FACE_MODEL_PATH))
    {
        RCLCPP_ERROR(rclcpp::get_logger("referee_node"), "failed to load face model: %s", FACE_MODEL_PATH);
    }

    rclcpp::spin(std::make_shared<RefereeNode>());
    rclcpp::shutdown();
    return 0;
}
