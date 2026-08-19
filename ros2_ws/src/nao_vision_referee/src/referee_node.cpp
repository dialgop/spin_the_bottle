// Simulates a few rounds of the game: each round, NAO looks down, watches a
// randomly picked recorded bottle-spin video in real time, and once the
// bottle settles, publishes the resulting head pose as a JointTrajectory for
// nao_webots_driver's head_controller to execute.
//
// If that lands on a valid target, also checks a randomly picked recorded
// face video for a face, the same way vision_standalone's
// face_detection_tests do - a stand-in for what NAO's camera would see after
// actually turning its head there, since there's no live/simulated camera
// yet (see ros2_ws/src/nao_webots_driver for why). This is only to test the
// wiring end-to-end; a real or simulated camera feed is the natural
// follow-up. Either way (no valid target, or no face found), NAO resets to
// a neutral "confused" pose instead.
//
// Each bottle-video frame is also republished as a plain sensor_msgs/Image
// while it's being watched, for nao_video_display's plugin to render onto
// the BottleScreen prop inside the simulation - so it's visible why NAO
// turned the way it did, not just that it did.
//
// The reusable logic (robot_gesture, settle_watcher) lives in separate
// header/src pairs so tests/referee_logic_tests.cpp can exercise it without
// a running ROS graph.
#include "face_detection.h"
#include "robot_gesture.h"
#include "settle_watcher.h"

#include <cv_bridge/cv_bridge.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <chrono>
#include <random>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{
    // One video per bottle-example recording - covers all 6, so a round can
    // land anywhere from a clean hit to the 225-315 degree dead zone.
    const std::vector<std::string> kBottleVideoNames = {
        "left_bottle_nao.mp4",  "left_down_bottle_nao.mp4",  "left_up_bottle_nao.mp4",
        "right_bottle_nao.mp4", "right_down_bottle_nao.mp4", "right_up_bottle_nao.mp4",
    };

    // One real person, one dog, one empty background - so a round with a
    // valid target still sometimes comes up "no one there".
    const std::vector<std::string> kFaceVideoNames = {
        "Man_surprised_nao.mp4",
        "Dog_happy_nao.mp4",
        "Background_no_person_nao.mp4",
    };
}

class RefereeNode : public rclcpp::Node
{
public:
    RefereeNode() : rclcpp::Node("referee_node"), rng_(std::random_device{}())
    {
        // Empty (the default) means "pick randomly each round" - only set
        // these to force the same video every round, for testing.
        declare_parameter<std::string>("bottle_video_path", "");
        declare_parameter<std::string>("face_video_path", "");
        declare_parameter<int>("rounds", kDefaultRounds);

        head_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/head_controller/joint_trajectory", 10);
        arm_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);
        bottle_video_publisher_ = create_publisher<sensor_msgs::msg::Image>("/bottle_video/image", 10);

        // Give head_controller/arm_controller a moment to come up before playing.
        timer_ = create_wall_timer(2s, std::bind(&RefereeNode::run_game, this));
    }

private:
    static constexpr int kDefaultRounds = 3;
    static constexpr double kMinWaitSeconds = 3.0;
    static constexpr double kMaxWaitSeconds = 5.0;

    void run_game()
    {
        timer_->cancel();

        const int rounds = get_parameter("rounds").as_int();
        for (int round = 1; round <= rounds; ++round)
        {
            RCLCPP_INFO(get_logger(), "--- round %d/%d ---", round, rounds);
            play_round();

            if (round < rounds)
            {
                const double wait_seconds = random_wait_seconds();
                RCLCPP_INFO(get_logger(), "waiting %.1fs before the next round", wait_seconds);
                std::this_thread::sleep_for(std::chrono::duration<double>(wait_seconds));
            }
        }
        RCLCPP_INFO(get_logger(), "--- game over ---");
    }

    void play_round()
    {
        publish_watching_head();

        const std::string video_path = pick_video_path("bottle_video_path", BOTTLE_EXAMPLES_DIR, kBottleVideoNames);
        const auto pose = settle_watcher::find_settled_head_pose(
            video_path, get_logger(), std::bind(&RefereeNode::publish_bottle_frame, this, std::placeholders::_1));
        if (!pose)
        {
            RCLCPP_WARN(get_logger(), "no head pose to publish - ask for the bottle to be spun again");
            publish_confused();
            return;
        }

        // Straight-ahead final pitch, not vision_standalone's real
        // HeadPose::pitch_radians: that value is calibrated for aiming a
        // real camera at seated-height faces, which doesn't apply yet (no
        // camera), and reads as a nod combined with the yaw turn.
        publish_head(pose->yaw_radians, 0.0);
        publish_pointing_arm(pose->yaw_radians);

        const std::string face_video_path = pick_video_path("face_video_path", FACE_EXAMPLES_DIR, kFaceVideoNames);
        if (settle_watcher::video_has_face(face_video_path, get_logger()))
        {
            RCLCPP_INFO(get_logger(), "detect_face -> found someone - ask them to spin next");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "detect_face -> no one there - ask the group to spin again");
            publish_confused();
        }
    }

    // Returns the fixed override from parameter_name if one was set,
    // otherwise a uniformly random pick from names within directory.
    std::string pick_video_path(const std::string& parameter_name, const std::string& directory,
                                 const std::vector<std::string>& names)
    {
        const std::string override_path = get_parameter(parameter_name).as_string();
        if (!override_path.empty())
        {
            return override_path;
        }
        std::uniform_int_distribution<size_t> distribution(0, names.size() - 1);
        return directory + "/" + names[distribution(rng_)];
    }

    double random_wait_seconds()
    {
        std::uniform_real_distribution<double> distribution(kMinWaitSeconds, kMaxWaitSeconds);
        return distribution(rng_);
    }

    // How long a publish_head_now trajectory point takes to physically
    // arrive. publish_watching_head blocks for this long afterwards, so the
    // head is actually down before find_settled_head_pose starts watching -
    // otherwise the video-watching phase would start while the head motor
    // is still mid-motion instead of strictly after it's in position.
    static constexpr double kHeadMoveSeconds = 1.0;

    // Looks down at the bottle and waits for that motion to actually
    // finish, so find_settled_head_pose's real-time watching phase starts
    // strictly after the head is in position, not concurrently with it.
    void publish_watching_head()
    {
        constexpr double kBottleWatchPitch = 0.4; // looking down at the bottle on the table
        publish_head_now(0.0, kBottleWatchPitch);
        std::this_thread::sleep_for(std::chrono::duration<double>(kHeadMoveSeconds));
    }

    // Lifts back to normal while turning to (yaw_radians, pitch_radians),
    // once find_settled_head_pose has actually finished watching.
    void publish_head(double yaw_radians, double pitch_radians)
    {
        publish_head_now(yaw_radians, pitch_radians);
    }

    void publish_head_now(double yaw_radians, double pitch_radians)
    {
        trajectory_msgs::msg::JointTrajectory message;
        message.joint_names = {"HeadYaw", "HeadPitch"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {yaw_radians, pitch_radians};
        point.time_from_start = rclcpp::Duration::from_seconds(kHeadMoveSeconds);
        message.points.push_back(point);

        RCLCPP_INFO(get_logger(), "publishing head pose: yaw=%f pitch=%f", yaw_radians, pitch_radians);
        head_publisher_->publish(message);
    }

    void publish_pointing_arm(double head_yaw_radians)
    {
        const robot_gesture::ArmPose arm_pose = robot_gesture::compute_arm_pose(head_yaw_radians);

        trajectory_msgs::msg::JointTrajectory message;
        message.joint_names = {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll",
                                "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {arm_pose.l_shoulder_pitch_radians, arm_pose.l_shoulder_roll_radians,
                            arm_pose.l_elbow_yaw_radians, arm_pose.l_elbow_roll_radians,
                            arm_pose.r_shoulder_pitch_radians, arm_pose.r_shoulder_roll_radians,
                            arm_pose.r_elbow_yaw_radians, arm_pose.r_elbow_roll_radians};
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);
        message.points.push_back(point);

        RCLCPP_INFO(get_logger(), "publishing arm pose: %s arm pointing", arm_pose.use_left_arm ? "left" : "right");
        arm_publisher_->publish(message);
    }

    // Resets to a neutral head and opens both arms, for "nowhere to point" -
    // either no valid target at all, or a valid target with no face there.
    void publish_confused()
    {
        publish_head(0.0, 0.0);

        using robot_gesture::kConfusedArmPose;

        trajectory_msgs::msg::JointTrajectory message;
        message.joint_names = {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll",
                                "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {kConfusedArmPose.shoulder_pitch_radians, kConfusedArmPose.shoulder_roll_magnitude_radians,
                            0.0, -kConfusedArmPose.elbow_roll_magnitude_radians,
                            kConfusedArmPose.shoulder_pitch_radians, -kConfusedArmPose.shoulder_roll_magnitude_radians,
                            0.0, kConfusedArmPose.elbow_roll_magnitude_radians};
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);
        message.points.push_back(point);

        RCLCPP_INFO(get_logger(), "publishing confused arm pose - opening both arms");
        arm_publisher_->publish(message);
    }

    // find_settled_head_pose's on_frame callback - republishes each bottle
    // video frame it reads so nao_video_display's plugin can render it onto
    // the BottleScreen prop, in sync with the real-time watching.
    void publish_bottle_frame(const cv::Mat& frame)
    {
        const auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        bottle_video_publisher_->publish(*message);
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr head_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bottle_video_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mt19937 rng_;
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
