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

    cv::Mat make_empty_frame()
    {
        return cv::Mat(480, 640, CV_8UC3, cv::Scalar(130, 125, 120));
    }

    cv::Mat make_bottle_frame(int radius, int shift_x = 0)
    {
        cv::Mat frame = make_empty_frame();
        cv::circle(frame, cv::Point(320 + shift_x, 300), radius, cv::Scalar(34, 177, 76), cv::FILLED);
        return frame;
    }
}

int main()
{
    check_state("bottle_detected on an empty frame",
                pre_game::bottle_detected(make_empty_frame()),
                pre_game::BottleState::NotFound);

    check_state("bottle_detected on a small (partial) bottle",
                pre_game::bottle_detected(make_bottle_frame(6)),
                pre_game::BottleState::OffCenter);

    check_state("bottle_detected on a full-size bottle",
                pre_game::bottle_detected(make_bottle_frame(40)),
                pre_game::BottleState::Centered);

    {
        const cv::Mat frame = make_bottle_frame(40);
        check(!pre_game::movement_detected(frame, frame),
              "movement_detected is false for two identical frames");
    }

    {
        const cv::Mat frame_a = make_bottle_frame(40);
        const cv::Mat frame_b = make_bottle_frame(40, 15);
        check(pre_game::movement_detected(frame_a, frame_b),
              "movement_detected is true when the bottle shifts position");
    }

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
