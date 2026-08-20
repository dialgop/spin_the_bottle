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
    if (!model_loaded)
    {
        return std::nullopt;
    }

    // training resized frames the same plain way via cv2.resize, so a
    // straight resize back is what matches the model's own preprocessing.
    const cv::Mat blob = cv::dnn::blobFromImage(
        bgr_image, 1.0 / 255.0, cv::Size(kInputSize, kInputSize), cv::Scalar(), /*swapRB=*/true, /*crop=*/false);
    net.setInput(blob);
    const cv::Mat output = net.forward();  // (1, 1, kInputSize, kInputSize) sigmoid probability map

    const cv::Mat prob_small(kInputSize, kInputSize, CV_32F,
                              const_cast<float*>(reinterpret_cast<const float*>(output.data)));
    cv::Mat prob;
    cv::resize(prob_small, prob, bgr_image.size());

    cv::Mat mask;
    cv::threshold(prob, mask, kProbThreshold, 255, cv::THRESH_BINARY);
    mask.convertTo(mask, CV_8U);

    if (cv::countNonZero(mask) < kMinMaskPixels)
    {
        return std::nullopt;
    }

    const cv::Rect bounds = cv::boundingRect(mask);
    const float confidence = static_cast<float>(cv::mean(prob, mask)[0]);

    return DetectedBottle{bounds, mask, confidence};
}

}
