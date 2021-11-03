#pragma once
#include "opencv2/video.hpp"
#include "image.h"

namespace habitat_cv {
    struct Video {
        Video(const std::string &file_name, const cv::Size &size, const std::string &codec = "mp4v");
        ~Video();
        void add_frame(const Image &);
        unsigned int frame_count;
        int fourcc;
        cv::VideoWriter writer;
    };

}