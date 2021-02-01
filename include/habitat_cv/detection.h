#pragma once
#include <habitat_cv/core.h>

namespace habitat_cv {
    struct Detection {
        Detection_location_list get_detections(cv::Mat &, Profile_list &);
    };
}