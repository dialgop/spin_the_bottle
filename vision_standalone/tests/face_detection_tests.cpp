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
    check(!face_detection::load_cascade("no_such_file.xml"),
          "load_cascade fails gracefully on a missing file");

    check(!face_detection::detect_face(make_blank_frame()).has_value(),
          "detect_face returns nullopt when no cascade is loaded");

    check(face_detection::load_cascade(FACE_CASCADE_PATH),
          "load_cascade succeeds on the checked-in cascade file");

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
