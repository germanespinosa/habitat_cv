#include <habitat_cv/core.h>

using namespace cell_world;

namespace habitat_cv {


    bool Profile::match(unsigned int area) {
        return area >= area_lower_bound && area <= area_upper_bound;
    }

    std::vector<Profile> Profile_list::match(unsigned int area) {
        vector <Profile> matches;
        for (auto &profile : *this)
            if (profile.match(area)) matches.push_back(profile);
        return matches;
    }

    Point::Point(double x, double y) : Location(x, y) {
    }

    Agent_info Frame_detection::to_agent_info() {
        Agent_info ai;
        ai.coordinates = this->detection_coordinates.coordinates;
        ai.location = this->detection_coordinates.detection_location.location;
<<<<<<< HEAD
//        ai.agent_name = this->detection_coordinates.detection_location.profile.agent_name;
=======
>>>>>>> 2348e1bbc129156473b4c8c7565ed3d16a3664d6
        ai.theta = this->theta;
        return ai;
    }
}
