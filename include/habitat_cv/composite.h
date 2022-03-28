#pragma once
#include <habitat_cv/image.h>
#include <cell_world.h>
#include <habitat_cv/camera_configuration.h>

namespace habitat_cv{
    struct Composite {
        Composite(Camera_configuration camera_configuration);
        Image &get_composite (const Images &);
        cell_world::Polygon &get_polygon(const cell_world::Coordinates &);
        cell_world::Coordinates get_coordinates(const cell_world::Location &);
        cv::Point2f get_raw_point(unsigned int camera_index, const cv::Point2f &);
        cv::Point2f get_warped_point(unsigned int camera_index, const cv::Point2f &);

        Image composite;
        Binary_image mask;
        cv::Size size;
        std::vector<cv::Mat> homographies;
        std::vector<cv::Mat> inverted_homographies;
        Images warped;
        std::vector<cv::Rect> crop_rectangles;
        cell_world::World world;
        cell_world::Map map;
        cell_world::Polygon_list cells;
        Camera_configuration configuration;
    };
}