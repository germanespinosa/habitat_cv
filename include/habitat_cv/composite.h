#pragma once
#include <cell_world.h>
#include <habitat_cv/camera_configuration.h>
#include <habitat_cv/image.h>

namespace habitat_cv{
    struct Composite {
        Composite(Camera_configuration camera_configuration, float resize_factor = 4);
        Image &get_composite (const Images &);
        cell_world::Polygon &get_polygon(const cell_world::Coordinates &);
        cell_world::Coordinates get_coordinates(const cell_world::Location &);
        Image composite;
        Image composite_large;
        Binary_image mask;
        Binary_image mask_large;
        cv::Size size;
        cv::Size size_large;
        std::vector<cv::Mat> homographies;
        Images warped;
        std::vector<cv::Rect> crop_rectangles;
        cell_world::World world;
        cell_world::World world_large;
        cell_world::Map map;
        cell_world::Map map_large;
        cell_world::Polygon_list cells;
        Camera_configuration configuration;
        float resize_factor;
    };
}