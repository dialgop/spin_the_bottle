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

}

int main()
{
    using world_coordinates::WorldPoint;
    using world_coordinates::HeadPose;
    using world_coordinates::find_target_point;
    using world_coordinates::compute_head_pose;
    using world_coordinates::to_world_coordinates;

    check_point("find_target_point general case, Case1.1 (45 degrees)",
                find_target_point(WorldPoint{0, 50, 45}),
                cv::Point2d(41.14, 91.14));

    check_point("find_target_point general case, Case1.2 (135 degrees)",
                find_target_point(WorldPoint{0, 50, 135}),
                cv::Point2d(-41.14, 91.14));

    check_point("find_target_point catch-all band (190 degrees)",
                find_target_point(WorldPoint{0, 50, 190}),
                cv::Point2d(-94.26, 33.37));

    check_point("find_target_point vertical case (90 degrees)",
                find_target_point(WorldPoint{0, 50, 90}),
                cv::Point2d(0, 100));

    check_point("find_target_point excluded band (270 degrees)",
                find_target_point(WorldPoint{0, 50, 270}),
                std::nullopt);

    check_world_point("to_world_coordinates (negative angle)",
                      to_world_coordinates(cv::Point2d(320, 300), -30, cv::Mat(480, 640, CV_8UC3)),
                      WorldPoint{0, 21, 30});

    check_world_point("to_world_coordinates (non-negative angle)",
                      to_world_coordinates(cv::Point2d(320, 300), 100, cv::Mat(480, 640, CV_8UC3)),
                      WorldPoint{0, 21, 260});

    const double kExpectedPitch = std::atan2(27.5, 100.0);

    if (failure_count > 0)
    {
        std::cout << failure_count << " check(s) FAILED\n";
        return 1;
    }

    std::cout << "all checks passed\n";
    return 0;
}
