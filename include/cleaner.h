#pragma once
#include <core.h>

namespace habitat_cv{
    struct Cleaner {
        cv::Mat &clean (cv::Mat &subtracted);
        cv::Mat clean_image;
    };
}