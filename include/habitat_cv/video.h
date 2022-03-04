#pragma once
#include "opencv2/video.hpp"
#include <habitat_cv/image.h>

namespace habitat_cv {
    struct Video {
        Video(const cv::Size &size, Image::Type type);
        ~Video();
        bool new_video(const std::string &file_name);
        bool close();
        bool add_frame(const Image &);
        bool is_open() const;
        static void set_fps(unsigned int);
        static unsigned int get_fps();
        int frame_count;
        cv::Size size;
        Image::Type type;
        int fourcc;
        cv::VideoWriter writer;
        std::mutex write_mutex;
    };

}