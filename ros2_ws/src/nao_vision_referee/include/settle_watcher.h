#pragma once

#include "world_coordinates.h"

#include <rclcpp/rclcpp.hpp>

#include <opencv2/core.hpp>

#include <functional>
#include <optional>
#include <string>

// Watches recorded footage in real time to decide where NAO should point
// and whether there's a face there - split out from referee_node.cpp so
// it's testable against real footage without needing a running ROS graph.
namespace settle_watcher
{
    // Steps through video_path frame by frame until the bottle has been
    // still for kSettleStreak consecutive frames after genuinely moving
    // (mirrors vision_standalone/src/main.cpp's per-frame pipeline), then
    // returns the head pose the latest frame points to. A real hand-spun
    // bottle can pause for a single frame mid-spin - requiring a streak
    // instead of just one still frame avoids grabbing that false stop
    // instead of where it actually ends up. Returns std::nullopt if the
    // video can't be opened, the bottle never settles, or the settled frame
    // doesn't yield a usable target - callers only need to know whether
    // there's somewhere to look.
    //
    // Frame reads are paced to the video's own frame rate, so this call
    // blocks for as long as the bottle actually took to settle in the
    // recording - a real robot watching a live feed couldn't know that
    // duration in advance either, so this doesn't precompute or fake it.
    //
    // If on_frame is set, it's called with every frame as it's read (before
    // the pacing sleep, after it's decoded), letting a caller show the same
    // footage elsewhere (e.g. nao_video_display) in sync with the real-time
    // watching - kept as a plain callback rather than a ROS publisher
    // directly, so this function stays testable without a running ROS graph.
    std::optional<world_coordinates::HeadPose> find_settled_head_pose(
        const std::string& video_path, const rclcpp::Logger& logger,
        const std::function<void(const cv::Mat&)>& on_frame = nullptr);

    // Scans every frame of video_path for a face, stopping at the first hit.
    // Mirrors vision_standalone's face_detection_tests any_frame_has_face
    // helper. face_detection::load_model must already have succeeded.
    bool video_has_face(const std::string& video_path, const rclcpp::Logger& logger);
}
