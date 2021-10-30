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

    struct Profile_list : json_cpp::Json_vector<Profile>{
        std::vector<Profile> match(unsigned int area);
    };

    struct Detection_location : json_cpp::Json_object{
        Json_object_members(
                Add_member(location);
                Add_member(area);
                Add_member(profile);
        );
        cell_world::Location location;
        unsigned int area;
        Profile profile;
    };

    using Detection_location_list = json_cpp::Json_vector<Detection_location> ;

    struct Detection_coordinates : json_cpp::Json_object{
        Json_object_members(
                Add_member(detection_location);
                Add_member(coordinates);
        );
        Detection_location detection_location;
        cell_world::Coordinates coordinates;
    };

    using Detection_coordinates_list = json_cpp::Json_vector<Detection_coordinates> ;

    struct Agent_info : json_cpp::Json_object{
        Json_object_members(
                Add_member(coordinates);
                Add_member(location);
                Add_member(theta);
                Add_member(agent_name);
        );
        cell_world::Coordinates coordinates;
        cell_world::Location location;
        double theta;
        std::string agent_name;
    };

    struct Frame_detection : json_cpp::Json_object{
        Json_object_members(
                Add_member(frame);
                Add_member(time_stamp);
                Add_member(theta);
                Add_member(detection_coordinates);
        )
        unsigned int frame;
        double time_stamp;
        double theta;
        Detection_coordinates detection_coordinates;
        Agent_info to_agent_info();
    };
}