#pragma once
#include <habitat_cv/core.h>
#include <cell_world.h>
#include <habitat_cv/camera_configuration.h>
#include <habitat_cv/image.h>

namespace habitat_cv{
    struct Composite {
        Composite(const Camera_configuration &camera_configuration);
        cv::Mat &get_composite (const Images &, bool = false);
        std::vector<cv::Point> get_hexagon(const cell_world::Coordinates &) const;
        void draw_cell(const cell_world::Coordinates &, const cv::Scalar = cv::Scalar(255,0,255));
        void draw_circle (const cell_world::Location &, int, const cv::Scalar = cv::Scalar(255,0,255));
        void draw_arrow(const cell_world::Location &, double, const cv::Scalar = cv::Scalar(255,0,255), double length = 1);
        cell_world::Coordinates get_coordinates(const cell_world::Location &);
        cell_world::Location get_location(const cell_world::Location &, unsigned int);
        float flip_y(double y) const;
        cv::Mat &get_rgb();
        Image composite;
        Image rgb_composite;
        cv::Size size;
        std::vector<cv::Mat> homographies;
        Images warped;
        std::vector<cv::Rect> crop_rectangles;
        cell_world::Coordinates_list valid_coordinates;
        cell_world::World world;
        cell_world::Map map;
        cell_world::Polygon_list cells;
        Camera_configuration configuration;
    private:
        cv::Point get_point(const cell_world::Coordinates &) const;
    };
}