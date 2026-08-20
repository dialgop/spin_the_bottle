#include "bottle_detector.h"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
    // Matches the IMG_SIZE the U-Net was trained/exported with.
    constexpr int kInputSize = 256;
    constexpr float kProbThreshold = 0.5f;

    // Filters out pure sensor/compression noise in the probability map -
    // not a judgment call on "is this really a bottle", the model's own
    // per-pixel probabilities already make that call.
    constexpr int kMinMaskPixels = 50;

    cv::dnn::Net net;
    bool model_loaded = false;
}

namespace bottle_detector
{

bool load_model(const std::string& model_path)
{
    try
    {
        net = cv::dnn::readNetFromONNX(model_path);
    }
    catch (const cv::Exception&)
    {
        model_loaded = false;
        return false;
    }

    model_loaded = !net.empty();
    return model_loaded;
}

std::optional<DetectedBottle> detect_bottle(const cv::Mat& bgr_image)
{
    return std::nullopt;
}

}
