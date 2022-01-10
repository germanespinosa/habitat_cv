#pragma once
#include <habitat_cv.h>

namespace habitat_cv {
    struct Camera_array {
        Camera_array(const std::string &, unsigned int);

        void capture();

        void reset();

        unsigned int camera_count;
        habitat_cv::Images images;

        ~Camera_array();

    private:
        std::string config_file;

        void open();

        void close();

        unsigned int frame_size;
    };
}