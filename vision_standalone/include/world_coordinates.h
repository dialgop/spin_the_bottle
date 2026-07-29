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

    struct HeadPose
    {
        double pitch_radians;
        double yaw_radians; // 0 = straight ahead - ready to send to NAO's HeadYaw joint directly
    };

}
