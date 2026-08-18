#include "robot_gesture.h"

#include <algorithm>
#include <cmath>

namespace robot_gesture
{
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
}
