// Hand-derived correctness checks for world_coordinates, run independently
// of the image pipeline - each check compares the function's output against
// a value worked out by hand (see comments), so a mistake in the geometry
// can't hide behind a pixel-space round trip through the rest of the vision
// pipeline. Exits non-zero if any check fails, so this can plug into ctest.
#include "world_coordinates.h"

#include <cmath>
#include <iostream>
#include <optional>

namespace
{
    int failure_count = 0;

    void check_point(const char* name, std::optional<cv::Point2d> actual, std::optional<cv::Point2d> expected)
    {
        constexpr double kTolerance = 0.01;
        const bool matches = actual.has_value() == expected.has_value() &&
            (!actual.has_value() ||
             (std::abs(actual->x - expected->x) < kTolerance && std::abs(actual->y - expected->y) < kTolerance));

        std::cout << (matches ? "PASS " : "FAIL ") << name << ": got ";
        if (actual) std::cout << "(" << actual->x << ", " << actual->y << ")";
        else std::cout << "nullopt";
        std::cout << ", expected ";
        if (expected) std::cout << "(" << expected->x << ", " << expected->y << ")";
        else std::cout << "nullopt";
        std::cout << '\n';

        if (!matches) ++failure_count;
    }

    void check_world_point(const char* name, world_coordinates::WorldPoint actual, world_coordinates::WorldPoint expected)
    {
        constexpr double kTolerance = 0.01;
        const bool matches = std::abs(actual.x - expected.x) < kTolerance &&
            std::abs(actual.y - expected.y) < kTolerance &&
            std::abs(actual.angle_degrees - expected.angle_degrees) < kTolerance;

        std::cout << (matches ? "PASS " : "FAIL ") << name << ": got (" << actual.x << ", " << actual.y
                  << ", " << actual.angle_degrees << "), expected (" << expected.x << ", " << expected.y
                  << ", " << expected.angle_degrees << ")\n";

        if (!matches) ++failure_count;
    }

    void check_head_pose(const char* name, world_coordinates::HeadPose actual, world_coordinates::HeadPose expected)
    {
        constexpr double kTolerance = 0.001;
        const bool matches = std::abs(actual.pitch_radians - expected.pitch_radians) < kTolerance &&
            std::abs(actual.yaw_radians - expected.yaw_radians) < kTolerance;

        std::cout << (matches ? "PASS " : "FAIL ") << name << ": got (pitch=" << actual.pitch_radians
                  << ", yaw=" << actual.yaw_radians << "), expected (pitch=" << expected.pitch_radians
                  << ", yaw=" << expected.yaw_radians << ")\n";

        if (!matches) ++failure_count;
    }

}

int main()
{
    using world_coordinates::WorldPoint;
    using world_coordinates::HeadPose;
    using world_coordinates::find_target_point;
    using world_coordinates::compute_head_pose;
    using world_coordinates::to_world_coordinates;

    // Line through (0,50) at 45 degrees, hand-solved against the circle
    // x^2+y^2=100^2: 2x^2+100x-7500=0 -> x=41.144 or x=-91.144; 45 degrees
    // picks the positive-x root -> y=50+1*(41.144-0)=91.144.
    check_point("find_target_point general case, Case1.1 (45 degrees)",
                find_target_point(WorldPoint{0, 50, 45}),
                cv::Point2d(41.14, 91.14));

    // Same line/circle but a 135 degree pointing angle - exercises the
    // OTHER root-selection branch (Case1.2, angle > 95 and < 180).
    // slope = tan(135) = -1, hand-solved: 2x^2-100x-7500=0 ->
    // x=91.144 or x=-41.144; 135 degrees picks the negative-x root.
    check_point("find_target_point general case, Case1.2 (135 degrees)",
                find_target_point(WorldPoint{0, 50, 135}),
                cv::Point2d(-41.14, 91.14));

    // 190 degrees falls in the catch-all band (180-205), which picks
    // whichever root is nearer to NAO rather than by sign. Hand-solved with
    // slope=tan(190)=tan(10): roots are approx (77.16, 63.61) and
    // (-94.26, 33.37) - the second has the smaller y and is in front.
    check_point("find_target_point catch-all band (190 degrees)",
                find_target_point(WorldPoint{0, 50, 190}),
                cv::Point2d(-94.26, 33.37));

    // Vertical line through x=0: y = sqrt(100^2 - 0^2) = 100.
    check_point("find_target_point vertical case (90 degrees)",
                find_target_point(WorldPoint{0, 50, 90}),
                cv::Point2d(0, 100));

    // Inside the excluded 205-335 degree band: no player NAO can resolve.
    check_point("find_target_point excluded band (270 degrees)",
                find_target_point(WorldPoint{0, 50, 270}),
                std::nullopt);

    // Pixel (320,300) in a 640x480 frame is dead center horizontally, 60px
    // above vertical center - hand-converts to world (0, 21). Angle -30
    // (already negative) should fold straight to 30.
    check_world_point("to_world_coordinates (negative angle)",
                      to_world_coordinates(cv::Point2d(320, 300), -30, cv::Mat(480, 640, CV_8UC3)),
                      WorldPoint{0, 21, 30});

    // Angle 100 (non-negative) should fold to 360-100=260.
    check_world_point("to_world_coordinates (non-negative angle)",
                      to_world_coordinates(cv::Point2d(320, 300), 100, cv::Mat(480, 640, CV_8UC3)),
                      WorldPoint{0, 21, 260});

    const double kExpectedPitch = std::atan2(27.5, 100.0);

    // atan2(100,100)=45 degrees -> bucket [36,72) -> snapped to 54 -> yaw=54-90=-36 degrees.
    check_head_pose("compute_head_pose bucket (45 degrees)",
                    compute_head_pose(cv::Point2d(100, 100)),
                    HeadPose{kExpectedPitch, -36.0 * M_PI / 180.0});

    // atan2(1,100)=~0.57 degrees -> bucket [0,36) -> snapped to 18 -> yaw=18-90=-72 degrees.
    check_head_pose("compute_head_pose bucket (near 0 degrees)",
                    compute_head_pose(cv::Point2d(100, 1)),
                    HeadPose{kExpectedPitch, -72.0 * M_PI / 180.0});

    // atan2(100,-100)=135 degrees -> bucket [108,144) -> snapped to 126 -> yaw=126-90=36 degrees.
    check_head_pose("compute_head_pose bucket (135 degrees)",
                    compute_head_pose(cv::Point2d(-100, 100)),
                    HeadPose{kExpectedPitch, 36.0 * M_PI / 180.0});

    // A point BEHIND NAO (negative y) should never reach compute_head_pose in
    // practice (find_target_point only returns points with y > 0)
    check_head_pose("compute_head_pose safety clamp (point behind NAO)",
                    compute_head_pose(cv::Point2d(100, -50)),
                    HeadPose{kExpectedPitch, -90.0 * M_PI / 180.0});

    if (failure_count > 0)
    {
        std::cout << failure_count << " check(s) FAILED\n";
        return 1;
    }

    std::cout << "all checks passed\n";
    return 0;
}
