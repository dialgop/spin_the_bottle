#pragma once

#include <opencv2/core.hpp>
#include <optional>

// Works out which way the bottle is pointing once pre_game has confirmed
// one is on the table: finds the bottle's bounding box and rough pointing
// side, fits an ellipse to it, and turns that into an angle and a line that
// can be drawn on the frame.
namespace line_projection
{
    // Which side of its own bounding box the bottle's pointing (thin) end
    // faces, before the ellipse angle refines it further.
    enum class PointingDirection
    {
        Left,
        Up,
        Right,
        Down
    };

    struct PointingArea
    {
        PointingDirection direction;
        int top;
        int bottom;
        int left;
        int right;
    };

    // Locates the bottle's bounding box in a BGR frame and estimates which
    // side it's pointing towards by comparing pixel counts on either half
    // of the box.
    PointingArea find_pointing_area(const cv::Mat& bgr_image);

    struct PointingLine
    {
        cv::RotatedRect ellipse;  // the raw fitted ellipse, for drawing
        double angle_degrees;     // corrected pointing angle, in degrees
        cv::Point2d line_end;     // point 100px out along angle_degrees
    };

    // Fits an ellipse to the bottle's silhouette and returns its center,
    // pointing angle (in degrees, corrected using `area.direction`) and a
    // point 100px out along that angle. Returns std::nullopt if the
    // silhouette doesn't have enough hull points to fit an ellipse to
    // (e.g. no bottle actually visible in bgr_image).
    std::optional<PointingLine> compute_pointing_line(const cv::Mat& bgr_image, const PointingArea& area);

    // Draws the ellipse, the pointing line and its two arrow "hooks" onto
    // image, in place. image is modified directly (no copy is made).
    void draw_pointing_line(cv::Mat& image, const PointingLine& line);
}