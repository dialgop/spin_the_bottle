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
        cascade_loaded = face_cascade.load(cascade_path);
        return cascade_loaded;
    }

    std::optional<DetectedFace> detect_face(const cv::Mat& bgr_image)
    {
        if (!cascade_loaded)
        {
            return std::nullopt;
        }

        cv::Mat gray;
        cv::cvtColor(bgr_image, gray, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(gray, gray);

        // Blank out the left/right quarters: a face there belongs to whichever
        // region NAO's neighbour is looking at, not the one it's currently
        // pointed at.
        const int left_bound = gray.cols / 4;
        const int right_bound = 3 * gray.cols / 4;
        gray(cv::Rect(0, 0, left_bound, gray.rows)).setTo(cv::Scalar(0));
        gray(cv::Rect(right_bound, 0, gray.cols - right_bound, gray.rows)).setTo(cv::Scalar(0));

        std::vector<cv::Rect> faces;
        face_cascade.detectMultiScale(gray, faces, kScaleFactor, kMinNeighbors, 0 | cv::CASCADE_SCALE_IMAGE, kMinFaceSize);

        if (faces.empty())
        {
            return std::nullopt;
        }

        return DetectedFace{faces[0]};
    }

}