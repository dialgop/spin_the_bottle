// Correctness checks for pre_game against synthetic frames with known
// bottle-pixel counts, so bottle_detected's threshold buckets and
// movement_detected's diff-threshold logic can be checked without a real
// camera. Exits non-zero if any check fails, so this can plug into ctest.
#include "pre_game.h"

#include <opencv2/imgproc.hpp>
#include <iostream>

namespace
{
    int failure_count = 0;

    void check(bool condition, const char* description)
    {
        std::cout << (condition ? "PASS " : "FAIL ") << description << '\n';
        if (!condition) ++failure_count;
    }

    void check_state(const char* name, pre_game::BottleState actual, pre_game::BottleState expected)
    {
        const bool matches = actual == expected;
        std::cout << (matches ? "PASS " : "FAIL ") << name << ": got " << static_cast<int>(actual)
                  << ", expected " << static_cast<int>(expected) << '\n';
        if (!matches) ++failure_count;
    }

    // A plain, low-saturation background with no bottle-hue pixels in it at
    // all - threshold_mask's hue band never matches, so this reads as "no
    // bottle" regardless of the actual hue (same background used by the
    // other vision_standalone tests).
    cv::Mat make_empty_frame()
    {
        return cv::Mat(480, 640, CV_8UC3, cv::Scalar(130, 125, 120));
    }

    // A green blob of the given radius, optionally shifted sideways, placed
    // inside threshold_mask's central band (row > rows/4, columns within
    // the middle two thirds) so it isn't cropped out. saturation_mask fills
    // the blob's convex hull, so its mask area is ~pi*radius^2 pixels.
    cv::Mat make_bottle_frame(int radius, int shift_x = 0)
    {
        cv::Mat frame = make_empty_frame();
        cv::circle(frame, cv::Point(320 + shift_x, 300), radius, cv::Scalar(34, 177, 76), cv::FILLED);
        return frame;
    }
}

int main()
{
    // bottle_detected: no bottle-hue pixels anywhere -> NotFound.
    check_state("bottle_detected on an empty frame",
                pre_game::bottle_detected(make_empty_frame()),
                pre_game::BottleState::NotFound);

    // bottle_detected: radius-6 circle -> mask area ~pi*36 =~ 113 pixels,
    // between kBottleIncomplete (20) and kBottleThreshold (200) -> OffCenter.
    check_state("bottle_detected on a small (partial) bottle",
                pre_game::bottle_detected(make_bottle_frame(6)),
                pre_game::BottleState::OffCenter);

    // bottle_detected: radius-40 circle -> mask area ~pi*1600 =~ 5027
    // pixels, comfortably over kBottleThreshold (200) -> Centered.
    check_state("bottle_detected on a full-size bottle",
                pre_game::bottle_detected(make_bottle_frame(40)),
                pre_game::BottleState::Centered);

    // movement_detected: identical frames -> zero pixels changed, so no
    // movement regardless of how many bottle pixels are present.
    {
        const cv::Mat frame = make_bottle_frame(40);
        check(!pre_game::movement_detected(frame, frame),
              "movement_detected is false for two identical frames");
    }

    // movement_detected: the same bottle shifted 15px sideways changes
    // ~2385 mask pixels (the two circles' symmetric difference), well over
    // previous_pixel_count/3 (~1676) -> registers as movement.
    {
        const cv::Mat frame_a = make_bottle_frame(40);
        const cv::Mat frame_b = make_bottle_frame(40, 15);
        check(pre_game::movement_detected(frame_a, frame_b),
              "movement_detected is true when the bottle shifts position");
    }

    // movement_detected: no bottle in either frame -> previous_pixel_count
    // is 0, which short-circuits to false regardless of the (zero) diff.
    {
        const cv::Mat frame = make_empty_frame();
        check(!pre_game::movement_detected(frame, frame),
              "movement_detected is false when there's no bottle to begin with");
    }

    if (failure_count > 0)
    {
        std::cout << failure_count << " check(s) FAILED\n";
        return 1;
    }

    std::cout << "all checks passed\n";
    return 0;
}
