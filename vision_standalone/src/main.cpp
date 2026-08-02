// Throwaway test harness: exercises pre_game's, line_projection's,
// world_coordinates's and face_detection's functions against either a
// synthetic frame or a real video file, so one can see them run without ROS
// or live NAO hardware.
#include "pre_game.h"
#include "line_projection.h"
#include "world_coordinates.h"
#include "face_detection.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <cmath>

namespace
{
    // Stand-in for a real NAO camera frame until getting one: a plain
    // background with a green "bottle" ellipse offset by bottle_shift_x, so
    // one can exercise the pipeline without hardware. Detection is hue-based
    // now, so the bottle just needs a hue inside the green band (with some
    // real saturation) and the background needs to fall outside it - a
    // muted, low-saturation background keeps it out regardless of hue.
    cv::Mat make_test_frame(int bottle_shift_x)
    {
        cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(130, 125, 120));
        cv::ellipse(frame,
                    cv::Point(320 + bottle_shift_x, 300),
                    cv::Size(30, 90),
                    0, 0, 360,
                    cv::Scalar(34, 177, 76),
                    cv::FILLED);
        return frame;
    }

    // Runs the full pipeline against a single frame and prints each stage's
    // output, prefixed with frame_index so a video's worth of frames stays
    // readable. previous_frame is only used for movement_detected's diff.
    void process_frame(const cv::Mat& frame, const cv::Mat& previous_frame, int frame_index)
    {
        const std::string prefix = "[frame " + std::to_string(frame_index) + "] ";

        const cv::Mat mask = pre_game::saturation_mask(frame);
        cv::imwrite("saturation_mask.png", mask);

        const pre_game::BottleState state = pre_game::bottle_detected(frame);
        std::cout << prefix << "bottle_detected -> " << static_cast<int>(state) << '\n';

        const bool moving = pre_game::movement_detected(frame, previous_frame);
        std::cout << prefix << "movement_detected -> " << std::boolalpha << moving << '\n';

        const line_projection::PointingArea area = line_projection::find_pointing_area(frame);
        std::cout << prefix << "find_pointing_area -> direction=" << static_cast<int>(area.direction)
                  << " top=" << area.top << " bottom=" << area.bottom
                  << " left=" << area.left << " right=" << area.right << '\n';

        if (const auto line = line_projection::compute_pointing_line(frame, area))
        {
            std::cout << prefix << "compute_pointing_line -> angle=" << line->angle_degrees << " degrees\n";

            cv::Mat annotated = frame.clone();
            line_projection::draw_pointing_line(annotated, *line);
            cv::imwrite("pointing_line.png", annotated);

            const world_coordinates::WorldPoint world_point =
                world_coordinates::to_world_coordinates(line->ellipse.center, line->angle_degrees, frame);
            std::cout << prefix << "to_world_coordinates -> x=" << world_point.x << " y=" << world_point.y
                      << " angle=" << world_point.angle_degrees << " degrees\n";

            if (const auto target = world_coordinates::find_target_point(world_point))
            {
                std::cout << prefix << "find_target_point -> (" << target->x << ", " << target->y << ")\n";

                const world_coordinates::HeadPose pose = world_coordinates::compute_head_pose(*target);
                std::cout << prefix << "compute_head_pose -> pitch=" << pose.pitch_radians
                          << " rad, yaw=" << pose.yaw_radians << " rad ("
                          << pose.yaw_radians * 180.0 / M_PI << " degrees)\n";

                if (const auto face = face_detection::detect_face(frame))
                {
                    std::cout << prefix << "detect_face -> found a face at (" << face->bounds.x << ", " << face->bounds.y << ")\n";
                }
                else
                {
                    std::cout << prefix << "detect_face -> no face found\n";
                }
            }
            else
            {
                std::cout << prefix << "find_target_point -> nullopt (ask for the bottle to be spun again)\n";
            }
        }
        else
        {
            std::cout << prefix << "compute_pointing_line -> not enough points to fit an ellipse\n";
        }
    }

    // Reads video_path as a video file and runs process_frame on every
    // consecutive pair of frames. Returns false if the file couldn't be opened.
    bool run_on_video(const std::string& video_path)
    {
        cv::VideoCapture capture(video_path);
        if (!capture.isOpened())
        {
            std::cerr << "failed to open video: " << video_path << '\n';
            return false;
        }

        cv::Mat previous_frame, frame;
        int frame_index = 0;
        while (capture.read(frame))
        {
            if (!previous_frame.empty())
            {
                process_frame(frame, previous_frame, frame_index);
            }
            previous_frame = frame.clone();
            ++frame_index;
        }

        std::cout << "processed " << frame_index << " frame(s) from " << video_path << '\n';
        return true;
    }
}

int main(int argc, char** argv)
{
    if (!face_detection::load_cascade(FACE_CASCADE_PATH))
    {
        std::cout << "face_detection::load_cascade -> failed to load " << FACE_CASCADE_PATH << '\n';
    }

    if (argc > 1)
    {
        return run_on_video(argv[1]) ? 0 : 1;
    }

    const cv::Mat frame_a = make_test_frame(0);
    const cv::Mat frame_b = make_test_frame(15);
    process_frame(frame_a, frame_b, 0);

    return 0;
}
