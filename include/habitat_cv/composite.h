#pragma once
#include <cell_world.h>
#include <habitat_cv/camera_configuration.h>
#include <habitat_cv/image.h>

namespace habitat_cv{
    struct Composite {
        Composite(const Camera_configuration &camera_configuration);
        Image &get_composite (const Images &);
        cell_world::Polygon &get_polygon(const cell_world::Coordinates &);
        cell_world::Coordinates get_coordinates(const cell_world::Location &);
        Image composite;
        Binary_image mask;
        cv::Size size;
        std::vector<cv::Mat> homographies;
        Images warped;
        std::vector<cv::Rect> crop_rectangles;
        cell_world::World world;
        cell_world::Map map;
        cell_world::Polygon_list cells;
        Camera_configuration configuration;
    };
}