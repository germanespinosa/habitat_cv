#pragma once
#include <habitat_cv/core.h>

namespace habitat_cv {
    struct Detection : json_cpp::Json_object {
        cell_world::Location location;
        unsigned int area;
    };

    struct Detection_list : json_cpp::Json_vector<Detection> {
        static Detection_list get_detections(cv::Mat &);
    private:
        Detection_list();
    };
}