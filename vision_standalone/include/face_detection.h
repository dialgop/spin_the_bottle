#pragma once

#include <opencv2/core.hpp>
#include <optional>
#include <string>

// Ports DetectFace.cpp's Haar-cascade check: once world_coordinates has NAO
// looking at a region, is there actually a person there? Runs OpenCV's Haar
// cascade on the frame's central half, since a face in the outer quarters
// belongs to whichever region NAO's neighbour is looking at, not this one.
namespace face_detection
{
    struct DetectedFace
    {
        cv::Rect bounds; // in frame pixel coordinates
    };

    // Loads the Haar-cascade classifier detect_face uses, from a path on
    // disk (e.g. "data/haarcascade_frontalface_alt.xml"). Must be called,
    // and must succeed, before detect_face can find anything - unlike the
    // 2015 version, this happens once rather than being reloaded on every
    // frame, and takes the path directly rather than resolving it through
    // a ROS package name. Returns false if the file couldn't be loaded.
    bool load_cascade(const std::string& cascade_path);

    // Runs the loaded cascade against bgr_image, restricted to its central
    // half (columns [cols/4, 3*cols/4)), and returns the first face found,
    // if any. Returns std::nullopt if no cascade was successfully loaded
    // first, or if no face was found - callers only need to know whether
    // someone can be seen, not which of those two reasons is why not.
    std::optional<DetectedFace> detect_face(const cv::Mat& bgr_image);
}