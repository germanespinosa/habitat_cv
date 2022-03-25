#pragma once
#include <habitat_cv/camera.h>
#include <thread>
namespace habitat_cv {
    struct Camera_array {
        Camera_array(const std::string &, unsigned int);
        Images capture();
        void reset();
        std::string config_file_path;
        unsigned int camera_count;
        ~Camera_array();
        std::vector<Camera *> cameras;
    };
}