#pragma once
#include <opencv2/opencv.hpp>

namespace maze_cv {
    struct Camera {
        virtual cv:Mat &Capture() = 0;
        cv::Mat image;
    };
}