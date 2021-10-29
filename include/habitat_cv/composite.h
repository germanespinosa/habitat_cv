#pragma once
#include <habitat_cv/core.h>
#include <cell_world.h>

namespace habitat_cv{
    struct Composite {
        Composite(const Camera_order &, const Cameras_associations &);
        cv::Mat &get_composite (const std::vector<cv::Mat> &, bool = false);
        cv::Point get_point(const cell_world::Coordinates &) const;
        std::vector<cv::Point> get_hexagon(const cell_world::Coordinates &) const;
        void draw_cell(cv::Mat &, const cell_world::Coordinates &, const cv::Scalar = cv::Scalar(255,0,255)) const;
        void draw_circle (cv::Mat &, const cv::Point &, int, const cv::Scalar = cv::Scalar(255,0,255)) const;
        void draw_arrow(cv::Mat &, const cv::Point &, double, const cv::Scalar = cv::Scalar(255,0,255)) const;
        cell_world::Coordinates get_coordinates(cv::Point);
        Point get_location(Point, unsigned int);

        float flip_y(double y) const;
        cv::Mat composite;
        Camera_order camera_order;
        cv::Size size;
        std::vector<cv::Mat> homographies;
        std::vector<cv::Mat> warped;
        std::vector<cv::Rect> crop_rectangles;
        cell_world::Coordinates_list valid_coordinates;
        cell_world::World world;
        cell_world::Map map;
        cell_world::Polygon_list cells;
    };
}