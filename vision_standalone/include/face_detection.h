#pragma once

#include <opencv2/core.hpp>
#include <optional>
#include <string>

// Ports DetectFace.cpp's face check: is there a person where NAO is looking?
// Runs on the frame's central half, since a face in the outer quarters
// belongs to NAO's neighbour's region, not this one.
//
// Uses YuNet (an ONNX model from the OpenCV Zoo) instead of the 2015 code's
// Haar cascade, which false-positived on a dog's face. Run via generic
// cv::dnn::readNetFromONNX/Net::forward with a decode of YuNet's outputs,
// not cv::FaceDetectorYN - that class's version in this project's OpenCV
//(4.6.0) crashes on this model.
namespace face_detection
{
    struct DetectedFace
    {
        cv::Rect bounds; // in frame pixel coordinates
    };

    // Loads the YuNet model detect_face uses, from a path on disk (e.g.
    // "data/face_detection_yunet_2023mar.onnx"). Must be called, and must
    // succeed, before detect_face can find anything - happens once rather
    // than being reloaded on every frame. Returns false if the file couldn't
    // be loaded, rather than throwing error.
    bool load_model(const std::string& model_path);

    // Runs the loaded model against bgr_image, restricted to its central
    // half (columns [cols/4, 3*cols/4)), and returns the highest-confidence
    // face found, if any. Returns std::nullopt if no model was successfully
    // loaded first, or if no face was found - callers only need to know
    // whether someone can be seen, not which of those two reasons is why not.
    std::optional<DetectedFace> detect_face(const cv::Mat& bgr_image);
}