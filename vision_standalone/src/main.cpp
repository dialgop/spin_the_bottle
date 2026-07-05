// Throwaway test harness: exercises pre_game's functions against a
// synthetic frame so one can see them run without a real camera or ROS.
#include "pre_game.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

namespace
{
    // Stand-in for a real NAO camera frame until getting one: a plain
    // saturated background with a dark "bottle" ellipse offset by
    // bottle_shift_x, so one can exercise the pipeline without hardware.
    cv::Mat make_test_frame(int bottle_shift_x)
    {
        cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(60, 140, 60));
        cv::ellipse(frame,
                    cv::Point(320 + bottle_shift_x, 300),
                    cv::Size(30, 90),
                    0, 0, 360,
                    cv::Scalar(20, 20, 20),
                    cv::FILLED);
        return frame;
    }
}

int main()
{
    const cv::Mat frame_a = make_test_frame(0);
    const cv::Mat frame_b = make_test_frame(15);

    const cv::Mat mask = pre_game::saturation_mask(frame_a);
    cv::imwrite("saturation_mask.png", mask);
    std::cout << "wrote saturation_mask.png\n";

    const pre_game::BottleState state = pre_game::bottle_detected(frame_a);
    std::cout << "bottle_detected -> " << static_cast<int>(state) << '\n';

    const bool moving = pre_game::movement_detected(frame_a, frame_b);
    std::cout << "movement_detected -> " << std::boolalpha << moving << '\n';

    return 0;
}
