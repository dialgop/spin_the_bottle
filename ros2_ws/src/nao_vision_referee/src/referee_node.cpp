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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>

using namespace std::chrono_literals;

namespace
{
    // Points whichever arm is on the same side as the head turn (positive
    // yaw = left, matching HeadPose's convention), by swinging that
    // shoulder outward proportionally to how far round the head turned, and
    // straightening its elbow; the other arm hangs down at the side, like a
    // normal standing pose, rather than held out in front. Not derived from
    // vision_standalone - this is robot-arm kinematics, not vision, so it
    // lives here rather than in the vision pipeline.
    //
    // NAO's elbow roll range never includes 0 on either side (left is
    // always negative, right always positive), unlike shoulder roll, so the
    // resting arm needs a real resting bend rather than a plain 0.0.
    struct ArmPose
    {
        bool use_left_arm;
        double l_shoulder_pitch_radians;
        double l_shoulder_roll_radians;
        double l_elbow_yaw_radians;
        double l_elbow_roll_radians;
        double r_shoulder_pitch_radians;
        double r_shoulder_roll_radians;
        double r_elbow_yaw_radians;
        double r_elbow_roll_radians;
    };

    ArmPose compute_arm_pose(double head_yaw_radians)
    {
        constexpr double kPointingShoulderPitch = -0.3; // raises the arm from resting (0 = horizontal forward)
        constexpr double kRestingShoulderPitch = 1.5;   // arm down at the side, like a normal standing pose
        constexpr double kMaxShoulderRoll = 1.0;        // stays inside NAO's real ~1.33 rad shoulder-roll range
        constexpr double kPointingElbowRoll = 0.2;      // near-straight, extended for pointing
        constexpr double kRestingElbowRoll = 0.3;       // gentle bend, arm relaxed at the side

        const bool use_left_arm = head_yaw_radians >= 0.0;
        const double roll_magnitude = std::min(std::abs(head_yaw_radians), kMaxShoulderRoll);

        ArmPose pose{};
        pose.use_left_arm = use_left_arm;
        pose.l_shoulder_pitch_radians = use_left_arm ? kPointingShoulderPitch : kRestingShoulderPitch;
        pose.l_shoulder_roll_radians = use_left_arm ? roll_magnitude : 0.0;
        pose.l_elbow_yaw_radians = 0.0;
        pose.l_elbow_roll_radians = use_left_arm ? -kPointingElbowRoll : -kRestingElbowRoll;
        pose.r_shoulder_pitch_radians = use_left_arm ? kRestingShoulderPitch : kPointingShoulderPitch;
        pose.r_shoulder_roll_radians = use_left_arm ? 0.0 : -roll_magnitude;
        pose.r_elbow_yaw_radians = 0.0;
        pose.r_elbow_roll_radians = use_left_arm ? kRestingElbowRoll : kPointingElbowRoll;

        return pose;
    }
    // Steps through video_path frame by frame until the bottle has been
    // still for kSettleStreak consecutive frames after genuinely moving
    // (mirrors vision_standalone/src/main.cpp's per-frame pipeline), then
    // returns the head pose the latest frame points to. A real hand-spun
    // bottle can pause for a single frame mid-spin - requiring a streak
    // instead of just one still frame avoids grabbing that false stop
    // instead of where it actually ends up. Returns std::nullopt if the
    // video can't be opened, the bottle never settles, or the settled frame
    // doesn't yield a usable target - callers only need to know whether
    // there's somewhere to look.
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

        cv::Mat previous_frame, frame;
        bool seen_movement = false;
        int still_streak = 0;
        int frame_index = 0;
        while (capture.read(frame))
        {
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

        trajectory_msgs::msg::JointTrajectory head_message;
        head_message.joint_names = {"HeadYaw", "HeadPitch"};

        trajectory_msgs::msg::JointTrajectoryPoint head_point;
        head_point.positions = {pose->yaw_radians, pose->pitch_radians};
        head_point.time_from_start = rclcpp::Duration::from_seconds(2.0);
        head_message.points.push_back(head_point);

        RCLCPP_INFO(get_logger(), "publishing head pose: yaw=%f pitch=%f", pose->yaw_radians, pose->pitch_radians);
        head_publisher_->publish(head_message);

        const ArmPose arm_pose = compute_arm_pose(pose->yaw_radians);

        trajectory_msgs::msg::JointTrajectory arm_message;
        arm_message.joint_names = {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll",
                                    "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"};

        trajectory_msgs::msg::JointTrajectoryPoint arm_point;
        arm_point.positions = {arm_pose.l_shoulder_pitch_radians, arm_pose.l_shoulder_roll_radians,
                                arm_pose.l_elbow_yaw_radians, arm_pose.l_elbow_roll_radians,
                                arm_pose.r_shoulder_pitch_radians, arm_pose.r_shoulder_roll_radians,
                                arm_pose.r_elbow_yaw_radians, arm_pose.r_elbow_roll_radians};
        arm_point.time_from_start = rclcpp::Duration::from_seconds(2.0);
        arm_message.points.push_back(arm_point);

        RCLCPP_INFO(get_logger(), "publishing arm pose: %s arm pointing", arm_pose.use_left_arm ? "left" : "right");
        arm_publisher_->publish(arm_message);

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

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr head_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_publisher_;
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
