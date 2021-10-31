#pragma once
#include <json_cpp.h>
#include <cell_world.h>
#include <opencv2/opencv.hpp>

namespace habitat_cv{
    cv::Point2f to_point(const cell_world::Location &l);
    void arrow(cv::Mat &, const cv::Point2f &, const cv::Point2f &, const cv::Scalar &, double size);

    struct Profile : json_cpp::Json_object {
        Json_object_members(
                Add_member(area_lower_bound);
                Add_member(area_upper_bound);
                );
        unsigned int area_lower_bound;
        unsigned int area_upper_bound;
        bool match(unsigned int area);
    };

}