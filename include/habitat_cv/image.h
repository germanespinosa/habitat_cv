#pragma once
#include <json_cpp.h>
#include <cell_world.h>
#include <opencv2/opencv.hpp>

namespace habitat_cv {

    struct Binary_image : cv::Mat {
        Binary_image(cv::MatExpr);
        Binary_image dilate(unsigned int);
        Binary_image erode(unsigned int);
    private:
        Binary_image(cv::Mat);
    };

    struct Image : cv::Mat {
        enum Type{
            rgb,
            gray
        };
        Image()=default;
        Image(int rows, int cols, Type type);
        Image(cv::Mat, std::string);
        Image to_rgb() const;
        Image to_gray() const;
        Binary_image threshold(unsigned char) const;
        std::string file_name;
        Type type;
    };

    struct Images : std::vector<Image> {
        static Images read(const std::string &);
        static Images read(const std::string &, const std::string &);
        static Images read(const std::string &, const std::vector<std::string> &);
        Images to_rgb() const;
        Images to_gray() const;
    };
};