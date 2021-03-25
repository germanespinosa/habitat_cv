#pragma once
#include <habitat_cv/core.h>
namespace habitat_cv {
    struct Bg_subtraction {
        Bg_subtraction();
        Bg_subtraction(cv::Mat &);
        void set_background (cv::Mat &);
        cv::Mat & subtract(cv::Mat & );
        cv::Mat background;
        cv::Mat subtracted;
    };
}