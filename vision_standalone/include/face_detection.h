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
    // disk (e.g. "data/haarcascade_frontalface_alt.xml").
    bool load_cascade(const std::string& cascade_path);

    // Runs the loaded cascade against bgr_image, restricted to its central
    // half (columns [cols/4, 3*cols/4)), and returns the first face found,
    // if any.
    std::optional<DetectedFace> detect_face(const cv::Mat& bgr_image);
}
