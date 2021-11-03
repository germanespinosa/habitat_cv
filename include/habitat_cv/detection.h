#pragma once
#include <habitat_cv/image.h>

namespace habitat_cv {

    struct Profile : json_cpp::Json_object {
        Json_object_members(
                Add_member(area_lower_bound);
                Add_member(area_upper_bound);
        );
        unsigned int area_lower_bound;
        unsigned int area_upper_bound;
    };

    struct Detection : json_cpp::Json_object {
        cell_world::Location location;
        unsigned int area;
    };

    struct Detection_list : json_cpp::Json_vector<Detection> {
        static Detection_list get_detections(const Image &, unsigned char threshold, int cleaning_cycles);
        Detection_list filter (const Profile &);
    private:
        Detection_list() = default;
    };
}