#pragma once
#include <habitat_cv/image.h>
#include <thread>
namespace habitat_cv {
    struct Camera_array {
        Camera_array(const std::string &, unsigned int);
        Images &capture();
        void reset();
        unsigned int camera_count;
        Images buffers;
        ~Camera_array();

    private:
        std::string config_file;
        void open();
        void close();
        unsigned int frame_size;
        long last = -1;
    };
}