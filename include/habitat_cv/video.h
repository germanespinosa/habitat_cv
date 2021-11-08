#pragma once
#include "opencv2/video.hpp"
#include "image.h"

namespace habitat_cv {
    struct Video {
        Video(const std::string &file_name, const cv::Size &size, Image::Type type);
        ~Video();
        void add_frame(const Image &);
        unsigned int frame_count;
        Image::Type type;
        int fourcc;
        cv::VideoWriter writer;
    };

}