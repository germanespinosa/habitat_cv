#pragma once
#include <json_cpp.h>
#include <cell_world.h>
#include <opencv2/opencv.hpp>

namespace habitat_cv{

    struct Point : cell_world::Location{
        Point () = default;
        Point (double , double );
        template<typename T> T to(){
            T point;
            point.x = x;
            point.y = y;
            return point;
        }
    };

    struct Profile : json_cpp::Json_object {
        Json_object_members(
                Add_member(agent_name);
                Add_member(area_lower_bound);
                Add_member(area_upper_bound);
                );
        std::string agent_name;
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
        Point location;
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

    struct Cell_association : json_cpp::Json_object{
        Json_object_members(
                Add_member(centroid);
                Add_member(cell_coordinates);
                Add_optional_member(trust);
                );
        Point centroid;
        cell_world::Coordinates cell_coordinates;
        bool trust{true};
    };

    struct Cell_association_list : json_cpp::Json_vector<Cell_association> {
        Cell_association &get_closest (Point &);
        Detection_coordinates_list get_detections_coordinates(Detection_location_list &);
        Cell_association_list filter (cell_world::Coordinates_list);
    };

    struct Cameras_associations : json_cpp::Json_vector<Cell_association_list> {
        Cameras_associations filter (cell_world::Coordinates_list);
    };

    struct Camera_order: json_cpp::Json_vector <json_cpp::Json_vector<unsigned int>> {
        unsigned int rows() const;
        unsigned int cols() const;
        unsigned int count() const ;
        cell_world::Coordinates get_camera_coordinates(unsigned int) const;
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
    };
}