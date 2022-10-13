#pragma once
#include <habitat_cv/image.h>

namespace habitat_cv {
    struct Background {
        void set_path(const std::string &);

        habitat_cv::Image composite;

        bool load();

        bool update(habitat_cv::Image);

        std::string path;
    };
}