#pragma once
#include "opencv2/video.hpp"
#include "image.h"

namespace habitat_cv {
    struct Video {
        Video(const cv::Size &size, Image::Type type);
        ~Video();
        bool new_video(const std::string &file_name);
        bool close();
        bool add_frame(const Image &);
        bool is_open() const;
        int frame_count;
        cv::Size size;
        Image::Type type;
        int fourcc;
        cv::VideoWriter writer;
        std::mutex write_mutex;
    };

}