// Correctness checks for face_detection's cascade-loading and
// region-cropping plumbing. Note what these deliberately don't cover: a
// Haar cascade is trained on real face patterns, so there's no cheap
// synthetic shape that will make detect_face actually find a face - a true
// positive check needs a real face image, which isn't available yet (same
// gap as the synthetic-frame harness in main.cpp - see the README's future
// work). These checks stick to what's verifiable without one: that loading
// behaves correctly, and that detect_face never reports a face where there
// isn't one. Exits non-zero if any check fails, so this can plug into ctest.
#include "face_detection.h"

#include <opencv2/imgproc.hpp>
#include <iostream>

#ifndef FACE_CASCADE_PATH
#error "FACE_CASCADE_PATH must be defined by CMake to the cascade xml path"
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
}

int main()
{
    // load_cascade: a path that doesn't exist should fail to load rather
    // than throwing or crashing.
    check(!face_detection::load_cascade("no_such_file.xml"),
          "load_cascade fails gracefully on a missing file");

    // detect_face: with no cascade successfully loaded, there's nothing to
    // match against, so this should report nullopt rather than crash.
    check(!face_detection::detect_face(make_blank_frame()).has_value(),
          "detect_face returns nullopt when no cascade is loaded");

    // load_cascade: the real cascade file (copied from the 2015 project's
    // src/haarcascade_frontalface_alt.xml into vision_standalone/data/)
    // should load successfully.
    check(face_detection::load_cascade(FACE_CASCADE_PATH),
          "load_cascade succeeds on the checked-in cascade file");

    // detect_face: a plain, featureless frame has nothing resembling a face
    // in it, so the loaded cascade shouldn't report a false positive.
    check(!face_detection::detect_face(make_blank_frame()).has_value(),
          "detect_face finds nothing in a blank frame");

    if (failure_count > 0)
    {
        std::cout << failure_count << " check(s) FAILED\n";
        return 1;
    }

    std::cout << "all checks passed\n";
    return 0;
}
