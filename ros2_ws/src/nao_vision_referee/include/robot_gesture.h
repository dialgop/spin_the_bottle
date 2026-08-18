#pragma once

// Robot-arm kinematics for NAO's pointing/confused gestures - not derived
// from vision_standalone, since this is about how the robot presents
// itself, not vision. Split out from referee_node.cpp so it's testable in
// isolation (pure math, no ROS/OpenCV involved).
namespace robot_gesture
{
    // Points whichever arm is on the same side as the head turn (positive
    // yaw = left, matching world_coordinates::HeadPose's convention), by
    // swinging that shoulder outward proportionally to how far round the
    // head turned, and straightening its elbow; the other arm hangs down at
    // the side, like a normal standing pose, rather than held out in front.
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

    ArmPose compute_arm_pose(double head_yaw_radians);

    // "Confused, please spin again" gesture: both arms raised and opened
    // outward symmetrically, unlike compute_arm_pose's one-points/one-rests
    // shape. Used when there's nowhere to point (no valid target, or no
    // face found there).
    struct ConfusedArmPose
    {
        double shoulder_pitch_radians;
        double shoulder_roll_magnitude_radians;
        double elbow_roll_magnitude_radians;
    };

    constexpr ConfusedArmPose kConfusedArmPose{-0.2, 0.6, 0.3};
}