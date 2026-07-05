#pragma once

#include <opencv2/core.hpp>

// Vision checks used before the actual game starts: is a bottle placed in
// view, is it roughly centered, and has it stopped moving.
namespace pre_game
{
    enum class BottleState
    {
        Centered,     // enough "bottle-colored" pixels found, roughly centered
        OffCenter,    // some pixels found, but too few - ask the user to recenter it
        NotFound,     // essentially no matching pixels - nothing there
        FullFrame     // the whole frame matched - treated as a special "game starting" case
    };

    // Builds a black/white mask that highlights the bottle's silhouette in a
    // BGR camera frame. Pixel value 255 = "this looks like it could be the
    // bottle", 0 = background. See pre_game.cpp for how the threshold values
    // were chosen; they are tuned to a specific table/lighting setup and will
    // likely need recalibrating for a new camera or scene.
    cv::Mat saturation_mask(const cv::Mat& bgr_image);

    // Runs saturation_mask() on a single frame and classifies the result
    // (see BottleState) by how many mask pixels came back.
    BottleState bottle_detected(const cv::Mat& bgr_image);

    // Compares the bottle masks of two frames and reports whether enough of
    // the mask changed between them to say the bottle is still moving.
    bool movement_detected(const cv::Mat& previous_frame, const cv::Mat& current_frame);
}