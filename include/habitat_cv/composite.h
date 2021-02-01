#pragma once
#include <habitat_cv/core.h>

namespace habitat_cv{
    struct Composite {
        Composite(const cv::Size, const Camera_order &, const Cameras_associations &);
        cv::Mat &get_composite (const std::vector<cv::Mat> &);
        cv::Point get_point(const cell_world::Coordinates &) const;
        cv::Mat composite;
        Camera_order camera_order;
        cv::Size size;
        std::vector<cv::Mat> homographies;
        std::vector<cv::Mat> warped;
        std::vector<cv::Rect> crop_rectangles;

    };
}