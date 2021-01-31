#include <bg_subtraction.h>

namespace habitat_cv{
    cv::Mat &Bg_subtraction::subtract(cv::Mat &image) {
        assert(image.size == background.size);
        cv::absdiff(image, background, subtracted);
        return subtracted;
    }

    void Bg_subtraction::set_background(cv::Mat &image) {
        cv::cvtColor(image, background, cv::COLOR_BGR2GRAY);
    }

    Bg_subtraction::Bg_subtraction(cv::Mat &image) {
        set_background(image);
    }
}
