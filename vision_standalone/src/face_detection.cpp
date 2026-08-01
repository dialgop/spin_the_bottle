#include "face_detection.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

namespace
{
    constexpr double kScaleFactor = 1.1;
    constexpr int kMinNeighbors = 3;
    const cv::Size kMinFaceSize(40, 40);

    cv::CascadeClassifier face_cascade;
    bool cascade_loaded = false;
}

namespace face_detection
{

bool load_cascade(const std::string& cascade_path)
{
    return true;
}

std::optional<DetectedFace> detect_face(const cv::Mat& bgr_image)
{
    return std::nullopt;
}

}