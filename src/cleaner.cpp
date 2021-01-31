#include <cleaner.h>

namespace habitat_cv{

    cv::Mat &Cleaner::clean(cv::Mat &subtracted) {
        cv::Mat bw = subtracted > 50;
        cv::Mat eroded;
        cv::erode(bw, eroded, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
        cv::Mat dilated;
        cv::dilate(eroded, dilated, cv::Mat(), cv::Point(-1, -1), 4, 1, 2);
        cv::erode(dilated, clean_image, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
        return clean_image;
    }
}
