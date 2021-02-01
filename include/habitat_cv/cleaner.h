#pragma once
#include <habitat_cv/core.h>

namespace habitat_cv{
    struct Cleaner {
        Cleaner(unsigned int threshold = 50, unsigned int erosions = 2);
        cv::Mat &clean (cv::Mat &subtracted);
        unsigned int threshold;
        unsigned int erosions;
        cv::Mat clean_image;
    };
}