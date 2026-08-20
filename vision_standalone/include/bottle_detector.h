#pragma once

#include <opencv2/core.hpp>
#include <optional>
#include <string>

// Segments the bottle using a small U-Net trained from scratch on cutouts
// pulled from this project's own bottle-example footage (rotated and
// composited onto varied backgrounds - see
// vision_standalone/data/bottle_detector_model.onnx), as a generalizing
// alternative to vision_common::threshold_mask's fixed green hue band.
//
// A pretrained YOLOv8-seg model was tried first and fine-tuned the same
// way, but its ONNX export's detection head uses ops this project's OpenCV
// (4.6.0 - the version ROS2 Jazzy's cv_bridge is built against) can't
// parse, regardless of export opset/simplify settings. A plain U-Net
// avoids that entirely: its ONNX graph is just Conv/BatchNorm/ReLU/Resize/
// Sigmoid, and its output is a single per-pixel probability map rather
// than a detection head needing NMS + mask-coefficient decoding.
//
// Run via generic cv::dnn::readNetFromONNX/Net::forward, the same pattern
// face_detection.cpp uses for YuNet.
namespace bottle_detector
{
    struct DetectedBottle
    {
        cv::Rect bounds;  // bounding box, in frame pixel coordinates
        cv::Mat mask;     // binary mask (0/255), same size as the input frame
        float confidence;
    };

    // Loads the ONNX model detect_bottle uses, from a path on disk (e.g.
    // "data/bottle_detector_model.onnx"). Must be called, and must succeed,
    // before detect_bottle can find anything - happens once rather than
    // being reloaded on every frame. Returns false if the file couldn't be
    // loaded, rather than throwing an error.
    bool load_model(const std::string& model_path);

    // Runs the loaded model against bgr_image and returns the
    // highest-confidence bottle detection, if any scores above the model's
    // confidence threshold. Returns std::nullopt if no model was
    // successfully loaded first, or if nothing was found.
    std::optional<DetectedBottle> detect_bottle(const cv::Mat& bgr_image);
}
