// Correctness checks for face_detection's cascade-loading, region-cropping
// plumbing, and real-footage detection accuracy. Exits non-zero if any check
// fails, so this can plug into ctest.
#include "face_detection.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <string>

#ifndef FACE_MODEL_PATH
#error "FACE_MODEL_PATH must be defined by CMake to the YuNet onnx model path"
#endif

#ifndef FACE_EXAMPLES_DIR
#error "FACE_EXAMPLES_DIR must be defined by CMake to the face examples directory"
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

    // Returns true if detect_face finds a face in at least one frame of the
    // video at video_path.
    bool any_frame_has_face(const std::string& video_path)
    {
        cv::VideoCapture capture(video_path);
        cv::Mat frame;
        while (capture.read(frame))
        {
            if (face_detection::detect_face(frame))
            {
                return true;
            }
        }
        return false;
    }
}

int main()
{
    // load_cascade: a path that doesn't exist should fail to load rather
    // than throwing or crashing.
    check(!face_detection::load_model("no_such_file.onnx"),
          "load_model fails gracefully on a missing file");

    // detect_face: with no cascade successfully loaded, there's nothing to
    // match against, so this should report nullopt rather than crash.
    check(!face_detection::detect_face(make_blank_frame()).has_value(),
          "detect_face returns nullopt when no model is loaded");

    // load_cascade: the real cascade file (copied from the 2015 project's
    // src/haarcascade_frontalface_alt.xml into vision_standalone/data/)
    // should load successfully.
    check(face_detection::load_cascade(FACE_CASCADE_PATH),
          "load_cascade succeeds on the checked-in cascade file");
    check(face_detection::load_model(FACE_MODEL_PATH),
          "load_model succeeds on the checked-in YuNet model");

    // detect_face: a plain, featureless frame has nothing resembling a face
    // in it, so the loaded cascade shouldn't report a false positive.
    check(!face_detection::detect_face(make_blank_frame()).has_value(),
          "detect_face finds nothing in a blank frame");

    // The checks below run against real recorded footage in
    // data/faces_examples/ (a synthetic AI-generated face from
    // thispersondoesnotexist.com, a dog, and an empty background - "Small
    // cacti with a white wall background" via rawpixel.com) - this is the
    // true-positive coverage a synthetic frame can't provide, since a Haar
    // cascade needs an actual face pattern to match against.
    check(any_frame_has_face(std::string(FACE_EXAMPLES_DIR) + "/Man_surprised_nao.mp4"),
          "detect_face finds a face somewhere in a real video of a human face");

    check(!any_frame_has_face(std::string(FACE_EXAMPLES_DIR) + "/Background_no_person_nao.mp4"),
          "detect_face finds nothing in a real video of an empty background");

    // This asserts true, not false: this Haar cascade is known to
    // false-positive on Dog_happy_nao.mp4, a weakness of Haar cascades in
    // general (their gradient-based features can match an animal face
    // front-on closely enough to trigger). Pinning down this known-bad
    // behavior explicitly - rather than leaving it as an unverified comment -
    // means this check starts failing the moment that changes (e.g. once the
    // planned ONNX-based replacement lands, see the README), which is exactly
    // when it should be flipped to expect false instead.
    check(any_frame_has_face(std::string(FACE_EXAMPLES_DIR) + "/Dog_happy_nao.mp4"),
          "detect_face false-positives on a dog's face (known Haar-cascade limitation, see README)");
    check(!any_frame_has_face(std::string(FACE_EXAMPLES_DIR) + "/Dog_happy_nao.mp4"),
          "detect_face doesn't mistake a dog's face for a human one");

    if (failure_count > 0)
    {
        std::cout << failure_count << " check(s) FAILED\n";
        return 1;
    }

    std::cout << "all checks passed\n";
    return 0;
}
