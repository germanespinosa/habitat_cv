#include <habitat_cv/util.h>

namespace habitat_cv {
    cv::Mat to_gray(const cv::Mat &image) {
        cv::Mat grey;
        cv::cvtColor(image, grey, cv::COLOR_BGR2GRAY);
        return grey;
    }
}