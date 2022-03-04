#pragma once
#include <cell_world.h>
#include <opencv2/opencv.hpp>
#include <habitat_cv/image.h>

namespace habitat_cv {
    struct Content : Image {
        Content (cv::Size, Image::Type);
    };

    struct Content_text : Content {
        Content_text (cv::Size, Image::Type, const cv::Scalar &color, const cv::Scalar &bgcolor, float font_size = 1, int halign = 0, int valign = 0);
        Content_text &operator=(const std::string &);
        cell_world::Location location;
        int halign, valign;
        float font_size;
        cv::Scalar color;
        cv::Scalar bgcolor;
    };

    struct Content_resize : Content{
        Content_resize (cv::Size, Image::Type type = Image::Type::rgb);
        Content_resize &operator=(const Image&);
    };

    struct Content_crop : Content {
        Content_crop (const cell_world::Location &bottom_left, const cell_world::Location &top_right, Image::Type type = Image::Type::rgb);
        Content_crop(const cell_world::Location &center, int square_size, Image image);
        Content_crop &operator=(const Image&);
    private:
        cell_world::Location bottom_left;
        cell_world::Location top_right;
    };

    struct Layout : Image {
        struct Place_holder {
            Place_holder (Content &, cell_world::Location );
            Content &content;
            cell_world::Location location;
        };
        Layout (int rows, int cols, Type type);
        void add_place_holder(Content &, const cell_world::Location &);
        virtual Image get_image();
        std::vector<Place_holder> place_holders;
    };
}
