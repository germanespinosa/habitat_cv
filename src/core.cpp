#include <habitat_cv/core.h>

using namespace cell_world;

namespace habitat_cv {

    cv::Point2f to_point(const Location &l){
        return {(float)l.x, (float)l.y};
    }

    void arrow(cv::Mat &img, const cv::Point2f &src, const cv::Point2f &dst, const cv::Scalar &color, double size){
        int thickness = 2;
        int lineType = cv::LINE_8;
        cv::line( img, src,  dst, color, thickness, lineType );
    }

    bool Profile::match(unsigned int area) {
        return area >= area_lower_bound && area <= area_upper_bound;
    }

    std::vector<Profile> Profile_list::match(unsigned int area) {
        vector <Profile> matches;
        for (auto &profile : *this)
            if (profile.match(area)) matches.push_back(profile);
        return matches;
    }

    Agent_info Frame_detection::to_agent_info() {
        Agent_info ai;
        ai.coordinates = this->detection_coordinates.coordinates;
        ai.location = this->detection_coordinates.detection_location.location;
        ai.theta = this->theta;
        return ai;
    }
}
