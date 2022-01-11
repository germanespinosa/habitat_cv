#pragma once
#include <json_cpp.h>
#include <cell_world.h>
#include <opencv2/opencv.hpp>

namespace habitat_cv {

    struct Binary_image : cv::Mat {
        Binary_image() = default;
        explicit Binary_image(cv::MatExpr);
        Binary_image dilate(unsigned int);
        Binary_image erode(unsigned int);
    private:
        explicit Binary_image(cv::Mat);
    };

    struct Image : cv::Mat {
        enum Type{
            rgb,
            gray
        };
        Image()=default;
        Image(cv::Size , Type type);
        Image(int rows, int cols, Type type);
        Image(cv::Mat, std::string);
        [[nodiscard]] Image to_rgb() const;
        [[nodiscard]] Image to_gray() const;
        [[nodiscard]] Binary_image threshold(unsigned char) const;
        std::string file_name;
        Type type;
        void save(const std::string &);
        void save(const std::string &, const std::string &);
        static Image read(const std::string &, const std::string &);
        void arrow (const cell_world::Location &, const cell_world::Location &, const cv::Scalar &color);
        void arrow (const cell_world::Location &, double, double, const cv::Scalar &color);
        void line (const cell_world::Location &, const cell_world::Location &, const cv::Scalar &color);
        void line (const cell_world::Location &, double, double, const cv::Scalar &color);
        void polygon (const cell_world::Polygon &, const cv::Scalar &color, bool filled = false);
        void circle (const cell_world::Location &, double, const cv::Scalar &color, bool filled = false);
        void text (const cell_world::Location &, const std::string &, const cv::Scalar &color, float size = 1, int halign = 0, int valign = 0);
        void clear();
        Image diff(const Image &) const;
        [[nodiscard]] cv::Point2f get_point(const cell_world::Location &) const;
        cell_world::Location get_location(const cv::Point2f &) const;
        Image mask(const Binary_image &);
    };

    struct Images : std::vector<Image> {
        static Images read(const std::string &);
        static Images read(const std::string &, const std::string &);
        static Images read(const std::string &, const std::vector<std::string> &);
        [[nodiscard]] Images to_rgb() const;
        [[nodiscard]] Images to_gray() const;
        void save(const std::string &, const std::vector<std::string> &);
        void save(const std::string &);
        Image &get(const std::string &);
        Images clone();
    };
}