#include <habitat_cv/bg_subtraction.h>

namespace habitat_cv{
    cv::Mat &Bg_subtraction::subtract(cv::Mat &image) {
        assert(image.size == background.size);
        cv::absdiff(image, background, subtracted);
        return subtracted;
    }

    void Bg_subtraction::set_background(cv::Mat &image) {
        background = image.clone();
    }

    Bg_subtraction::Bg_subtraction(cv::Mat &image) {
        set_background(image);
    }

    Bg_subtraction::Bg_subtraction() {

    }
}
