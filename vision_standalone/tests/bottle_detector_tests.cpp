// Correctness checks for bottle_detector's model-loading plumbing and
// real-footage detection accuracy. Exits non-zero if any check fails, so
// this can plug into ctest.
#include "bottle_detector.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <string>
#include <vector>

#ifndef BOTTLE_MODEL_PATH
#error "BOTTLE_MODEL_PATH must be defined by CMake to the fine-tuned onnx model path"
#endif

#ifndef BOTTLE_EXAMPLES_DIR
#error "BOTTLE_EXAMPLES_DIR must be defined by CMake to the bottle examples directory"
#endif

namespace
{
    int failure_count = 0;

    void check(bool condition, const char* description)
    {
        std::cout << (condition ? "PASS " : "FAIL ") << description << '\n';
        if (!condition) ++failure_count;
    }

    cv::Mat make_blank_frame()
    {
        return cv::Mat(480, 640, CV_8UC3, cv::Scalar(130, 125, 120));
    }

    // Returns true if detect_bottle finds a bottle in at least one frame of
    // the video at video_path, with a mask that actually covers a plausible
    // amount of the frame (rules out a degenerate near-empty mask passing
    // the confidence check alone).
    bool any_frame_has_bottle(const std::string& video_path)
    {
        cv::VideoCapture capture(video_path);
        cv::Mat frame;
        while (capture.read(frame))
        {
            const auto detection = bottle_detector::detect_bottle(frame);
            if (detection && cv::countNonZero(detection->mask) > 500)
            {
                return true;
            }
        }
        return false;
    }
}

int main()
{
    // load_model: a path that doesn't exist should fail to load rather
    // than throwing or crashing.
    check(!bottle_detector::load_model("no_such_file.onnx"),
          "load_model fails gracefully on a missing file");

    // detect_bottle: with no model successfully loaded, there's nothing to
    // match against, so this should report nullopt rather than crash.
    check(!bottle_detector::detect_bottle(make_blank_frame()).has_value(),
          "detect_bottle returns nullopt when no model is loaded");

    // load_model: the fine-tuned model checked into vision_standalone/data/
    // should load successfully.
    check(bottle_detector::load_model(BOTTLE_MODEL_PATH),
          "load_model succeeds on the checked-in fine-tuned model");

    // detect_bottle: a plain, featureless frame has nothing resembling a
    // bottle in it, so the loaded model shouldn't report a false positive.
    check(!bottle_detector::detect_bottle(make_blank_frame()).has_value(),
          "detect_bottle finds nothing in a blank frame");

    // True-positive coverage against every recorded bottle-spin video this
    // project has - the model was fine-tuned on cutouts from these same
    // clips, so it should confidently find the bottle in all of them.
    const std::vector<std::string> videos = {
        "left_bottle_nao.mp4", "left_down_bottle_nao.mp4", "left_up_bottle_nao.mp4",
        "right_bottle_nao.mp4", "right_down_bottle_nao.mp4", "right_up_bottle_nao.mp4",
        "right_bottle_blue_nao.mp4", "right_up_gray_nao.mp4", "right_up_nao.mp4",
        "up_bottle_nao.mp4", "up_long_nao.mp4", "up_right_nao.mp4",
    };
    for (const auto& video : videos)
    {
        check(any_frame_has_bottle(std::string(BOTTLE_EXAMPLES_DIR) + "/" + video),
              ("detect_bottle finds the bottle somewhere in " + video).c_str());
    }

    if (failure_count > 0)
    {
        std::cout << failure_count << " check(s) FAILED\n";
        return 1;
    }

    std::cout << "all checks passed\n";
    return 0;
}