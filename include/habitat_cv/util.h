#pragma once
#include <opencv2/opencv.hpp>

namespace habitat_cv{
    cv::Mat to_gray(const cv::Mat &);
    bool file_exists(const std::string &);
    std::vector<cv::Mat> read_images(const std::string &path, const std::vector<std::string> &files);
    void write_images(const std::vector<cv::Mat> &images, const std::string &path, const std::vector<std::string> &files);
};