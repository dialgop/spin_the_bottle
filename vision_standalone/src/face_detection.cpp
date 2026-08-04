#include "face_detection.h"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace
{
    // YuNet's own recommended defaults (see the OpenCV Zoo sample code).
    constexpr float kScoreThreshold = 0.9f;
    constexpr float kNmsThreshold = 0.3f;
    constexpr int kTopK = 5000;

    // YuNet's 3 detection heads run on feature maps downsampled by these
    // factors from the (padded) input; each head's output naming follows
    // "<kind>_<stride>", e.g. "cls_8", matching the model's own output names.
    const std::vector<int> kStrides = {8, 16, 32};
    constexpr int kPadDivisor = 32;

    cv::dnn::Net net;
    bool model_loaded = false;

    // YuNet needs its input's dimensions to be multiples of 32; real camera
    // frames (and these recordings) are already 640x480, but this keeps
    // detect_face correct for any frame size.
    cv::Mat pad_to_divisor(const cv::Mat& image)
    {
        const int padded_w = ((image.cols - 1) / kPadDivisor + 1) * kPadDivisor;
        const int padded_h = ((image.rows - 1) / kPadDivisor + 1) * kPadDivisor;

        cv::Mat padded;
        cv::copyMakeBorder(image, padded, 0, padded_h - image.rows, 0, padded_w - image.cols,
                            cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        return padded;
    }

    // Ports FaceDetectorYNImpl::postProcess from OpenCV's own
    // modules/objdetect/src/face_detect.cpp: decodes the 3 detection heads'
    // raw output tensors into boxes + scores, then runs NMS. Only the
    // bounding box is kept, since DetectedFace doesn't need landmarks.
    std::optional<cv::Rect> decode_best_face(const std::vector<cv::Mat>& output_blobs, int padded_w, int padded_h)
    {
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;

        for (size_t i = 0; i < kStrides.size(); ++i)
        {
            const int stride = kStrides[i];
            const int cols = padded_w / stride;
            const int rows = padded_h / stride;

            const float* cls_v = reinterpret_cast<const float*>(output_blobs[i].data);
            const float* obj_v = reinterpret_cast<const float*>(output_blobs[i + kStrides.size()].data);
            const float* bbox_v = reinterpret_cast<const float*>(output_blobs[i + kStrides.size() * 2].data);

            for (int r = 0; r < rows; ++r)
            {
                for (int c = 0; c < cols; ++c)
                {
                    const size_t idx = static_cast<size_t>(r) * cols + c;

                    const float cls_score = std::clamp(cls_v[idx], 0.f, 1.f);
                    const float obj_score = std::clamp(obj_v[idx], 0.f, 1.f);
                    const float score = std::sqrt(cls_score * obj_score);

                    if (score < kScoreThreshold)
                    {
                        continue;
                    }

                    const float cx = (c + bbox_v[idx * 4 + 0]) * stride;
                    const float cy = (r + bbox_v[idx * 4 + 1]) * stride;
                    const float w = std::exp(bbox_v[idx * 4 + 2]) * stride;
                    const float h = std::exp(bbox_v[idx * 4 + 3]) * stride;

                    boxes.emplace_back(cv::Rect2f(cx - w / 2.f, cy - h / 2.f, w, h));
                    scores.push_back(score);
                }
            }
        }

        if (boxes.empty())
        {
            return std::nullopt;
        }

        std::vector<int> keep_idx;
        cv::dnn::NMSBoxes(boxes, scores, kScoreThreshold, kNmsThreshold, keep_idx, 1.f, kTopK);

        if (keep_idx.empty())
        {
            return std::nullopt;
        }

        const int best_idx = *std::max_element(keep_idx.begin(), keep_idx.end(),
            [&scores](int a, int b) { return scores[a] < scores[b]; });

        return boxes[best_idx];
    }
}

namespace face_detection
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

std::optional<DetectedFace> detect_face(const cv::Mat& bgr_image)
{
    if (!model_loaded)
    {
        return std::nullopt;
    }

    // Blank out the left/right quarters: a face there belongs to whichever
    // region NAO's neighbour is looking at, not the one it's currently
    // pointed at.
    cv::Mat cropped = bgr_image.clone();
    const int left_bound = cropped.cols / 4;
    const int right_bound = 3 * cropped.cols / 4;
    cropped(cv::Rect(0, 0, left_bound, cropped.rows)).setTo(cv::Scalar(0, 0, 0));
    cropped(cv::Rect(right_bound, 0, cropped.cols - right_bound, cropped.rows)).setTo(cv::Scalar(0, 0, 0));

    const cv::Mat padded = pad_to_divisor(cropped);

    static const std::vector<std::string> kOutputNames = {
        "cls_8", "cls_16", "cls_32", "obj_8", "obj_16", "obj_32",
        "bbox_8", "bbox_16", "bbox_32", "kps_8", "kps_16", "kps_32"};

    net.setInput(cv::dnn::blobFromImage(padded));

    std::vector<cv::Mat> output_blobs;
    net.forward(output_blobs, kOutputNames);

    const auto bounds = decode_best_face(output_blobs, padded.cols, padded.rows);
    if (!bounds)
    {
        return std::nullopt;
    }

    return DetectedFace{*bounds};
}

}