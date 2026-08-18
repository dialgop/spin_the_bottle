// Correctness checks for robot_gesture's arm-pose math and settle_watcher's
// real-footage decisions. Exits non-zero if any check fails, so this can
// plug into ctest, same style as vision_standalone's own tests.
#include "face_detection.h"
#include "robot_gesture.h"
#include "settle_watcher.h"

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <string>

#ifndef FACE_MODEL_PATH
#error "FACE_MODEL_PATH must be defined by CMake to the YuNet onnx model path"
#endif
#ifndef BOTTLE_EXAMPLES_DIR
#error "BOTTLE_EXAMPLES_DIR must be defined by CMake to the bottle examples directory"
#endif
#ifndef FACE_EXAMPLES_DIR
#error "FACE_EXAMPLES_DIR must be defined by CMake to the face examples directory"
#endif

namespace
{
    int failure_count = 0;

    void check(bool condition, const std::string& description)
    {
        std::cout << (condition ? "PASS " : "FAIL ") << description << '\n';
        if (!condition) ++failure_count;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const rclcpp::Logger logger = rclcpp::get_logger("referee_logic_tests");

    // compute_arm_pose: which arm points, and that shoulder roll is clamped
    // to stay inside NAO's real ~1.33 rad range rather than following an
    // arbitrarily large head yaw.
    {
        const auto pose = robot_gesture::compute_arm_pose(0.5);
        check(pose.use_left_arm, "compute_arm_pose points the left arm for positive yaw");
        check(pose.l_shoulder_roll_radians == 0.5, "compute_arm_pose's left shoulder roll matches a moderate yaw");
        check(pose.r_shoulder_roll_radians == 0.0, "compute_arm_pose rests the right shoulder roll at 0 while pointing left");
    }
    {
        const auto pose = robot_gesture::compute_arm_pose(-0.5);
        check(!pose.use_left_arm, "compute_arm_pose points the right arm for negative yaw");
        check(pose.r_shoulder_roll_radians == -0.5, "compute_arm_pose's right shoulder roll matches a moderate yaw");
    }
    {
        const auto pose = robot_gesture::compute_arm_pose(2.0);
        check(pose.l_shoulder_roll_radians == 1.0, "compute_arm_pose clamps shoulder roll instead of following a large yaw");
    }
    {
        const auto pose = robot_gesture::compute_arm_pose(0.0);
        check(pose.use_left_arm, "compute_arm_pose treats a yaw of exactly 0 as pointing left");
    }

    // find_settled_head_pose: against all 6 real recorded bottle videos.
    // left_bottle/left_up should both point left, right_bottle should point
    // right, and left_down/right_down should land in the 225-315 degree
    // dead zone and be rejected - this is a real regression check: an
    // earlier version of this settle-detection logic got left_up and
    // right_bottle's directions backwards by grabbing a false mid-spin
    // pause instead of where the bottle actually ended up.
    {
        const auto pose = settle_watcher::find_settled_head_pose(std::string(BOTTLE_EXAMPLES_DIR) + "/left_bottle_nao.mp4", logger);
        check(pose.has_value() && pose->yaw_radians > 0, "left_bottle_nao.mp4 settles pointing left");
    }
    {
        const auto pose = settle_watcher::find_settled_head_pose(std::string(BOTTLE_EXAMPLES_DIR) + "/left_up_bottle_nao.mp4", logger);
        check(pose.has_value() && pose->yaw_radians > 0, "left_up_bottle_nao.mp4 settles pointing left");
    }
    {
        const auto pose = settle_watcher::find_settled_head_pose(std::string(BOTTLE_EXAMPLES_DIR) + "/right_bottle_nao.mp4", logger);
        check(pose.has_value() && pose->yaw_radians < 0, "right_bottle_nao.mp4 settles pointing right");
    }
    {
        const auto pose = settle_watcher::find_settled_head_pose(std::string(BOTTLE_EXAMPLES_DIR) + "/left_down_bottle_nao.mp4", logger);
        check(!pose.has_value(), "left_down_bottle_nao.mp4 settles in the dead zone and is rejected");
    }
    {
        const auto pose = settle_watcher::find_settled_head_pose(std::string(BOTTLE_EXAMPLES_DIR) + "/right_down_bottle_nao.mp4", logger);
        check(!pose.has_value(), "right_down_bottle_nao.mp4 settles in the dead zone and is rejected");
    }

    // video_has_face: same 3 real videos vision_standalone's own tests use.
    check(face_detection::load_model(FACE_MODEL_PATH), "load_model succeeds on the checked-in YuNet model");

    check(settle_watcher::video_has_face(std::string(FACE_EXAMPLES_DIR) + "/Man_surprised_nao.mp4", logger),
          "video_has_face finds a face in a real video of a human face");
    check(!settle_watcher::video_has_face(std::string(FACE_EXAMPLES_DIR) + "/Dog_happy_nao.mp4", logger),
          "video_has_face doesn't mistake a dog's face for a human one");
    check(!settle_watcher::video_has_face(std::string(FACE_EXAMPLES_DIR) + "/Background_no_person_nao.mp4", logger),
          "video_has_face finds nothing in a real video of an empty background");

    rclcpp::shutdown();

    if (failure_count > 0)
    {
        std::cout << failure_count << " check(s) FAILED\n";
        return 1;
    }

    std::cout << "all checks passed\n";
    return 0;
}