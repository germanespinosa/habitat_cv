#pragma once
#include "opencv2/video.hpp"
#include <habitat_cv/image.h>
#include <thread>

namespace habitat_cv {
    struct Video {

        const std::string extension = ".mp4";
        const int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');

        Video(const cv::Size &size, Image::Type type);
        ~Video();
        bool new_video(const std::string &file_name);
        bool close();
        bool add_frame(const Image &);
        bool is_open() const;
        static void set_fps(unsigned int);
        static unsigned int get_fps();
        std::string file_name;
        int frame_count;
        cv::Size size;
        Image::Type type;
        cv::VideoWriter writer;
        std::thread writer_thread;
        std::queue<Image> pending_frames;
        std::atomic<bool> running;
        void split_video(const std::vector<cv::Point2f> &, const cv::Size &);
    };

}