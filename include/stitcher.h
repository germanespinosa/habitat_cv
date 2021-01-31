#pragma once
#include <json_cpp.h>
#include <cell_world.h>
#include <opencv2/opencv.hpp>

namespace maze_cv {
    struct Stitcher_parameters : json_cpp::Json_object {
        Json_object_members(
                Add_member(key_points);
                Add_member(width);
                Add_member(height);
                Add_member(camera_order);
        )

        cell_world::Coordinates_list key_points;
        double width;
        double height;
        json_cpp::Json_vector<json_cpp::Json_vector<unsigned int>> camera_order;

        bool is_key_point(cell_world::Coordinates &);
    };

    struct Cell_association : json_cpp::Json_object {
        Json_object_members(
                Add_member(centroid);
                Add_member(cell_coordinates);
                Add_member(trust);
        )

        cell_world::Location centroid;
        cell_world::Coordinates cell_coordinates;
        bool trust;
    };

    using Cell_association_list = json_cpp::Json_vector<Cell_association>;

    struct Stitcher {
        Stitcher(const std::string &, const std::string &);

        cv::Point2f create_point(const cell_world::Coordinates &) const;

        cv::Mat get_composite(std::vector<cv::Mat> &) const;

        cv::Rect get_crop_rectangle(unsigned int);

        Stitcher_parameters parameters;
        json_cpp::Json_vector<Cell_association_list> camera_associations;
        std::vector<cv::Mat> homographies;
        unsigned int rows, cols;
        cv::Size crop_size;
        std::vector<cv::Rect> crop_rectangles;
    };
};