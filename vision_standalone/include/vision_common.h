#pragma once

#include <opencv2/core.hpp>
#include <vector>

// Small pieces shared by pre_game and line_projection so the same
// "find the bottle-colored blob" logic isn't duplicated between them.
namespace vision_common
{
    // Same raw black/white mask that pre_game::saturation_mask builds on top
    // of, but WITHOUT filling in the convex hull. Some callers need the
    // unfilled version (e.g. to fit an ellipse to just the outline).
    //
    // Matches on Hue rather than brightness/saturation: the game uses a
    // green bottle (a wine bottle or a Sprite bottle both work), and Hue is
    // the one HSV component that stays roughly stable even when lighting
    // washes out Saturation/Value. The hue band and saturation floor below
    // are tuned for that green bottle and will need recalibrating for a
    // different bottle color.
    cv::Mat threshold_mask(const cv::Mat& bgr_image);

    // Finds contours in a binary mask, merges every contour point together,
    // and returns the convex hull enclosing all of them. Returns an empty
    // vector if the mask has no foreground pixels at all - callers must
    // check for that before doing anything (like fitEllipse) that requires
    // a minimum number of points.
    std::vector<cv::Point> convex_hull_of(const cv::Mat& binary_mask);
}
