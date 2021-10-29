#include <habitat_cv/core.h>

using namespace cell_world;

namespace habitat_cv {
    Cell_association &Cell_association_list::get_closest(Point &point) {
        double min_distance = 0;
        int closest = Not_found;
        for (int i = 0; i < this->size(); i++) {
            double distance = (*this)[i].centroid.dist(point);
            if (closest == Not_found || min_distance > distance) {
                min_distance = distance;
                closest = i;
            }
        }
        return (*this)[closest];
    }

    Cell_association_list Cell_association_list::filter(cell_world::Coordinates_list criteria) {
        Cell_association_list filtered;
        for (auto &ca : *this) {
            for (auto &cc : criteria)
                if (ca.cell_coordinates == cc) {
                    filtered.emplace_back(ca);
                    break;
                }
        }
        return filtered;
    }

    bool Profile::match(unsigned int area) {
        return area >= area_lower_bound && area <= area_upper_bound;
    }

    unsigned int Camera_order::rows() const {
        return this->size();
    }

    unsigned int Camera_order::cols() const {
        unsigned int cols_count = 0;
        for (auto &row : *this)
            if (row.size() > cols_count) cols_count = row.size();
        return cols_count;
    }

    Coordinates Camera_order::get_camera_coordinates(unsigned int camera) const {
        for (int r = 0; r < rows(); r++)
            for (int c = 0; c < (*this).size(); c++)
                if ((*this)[r][c] == camera) return Coordinates{c, r};
        return Coordinates{-1, -1};
    }

    unsigned int Camera_order::count() const {
        unsigned int max = 0;
        for (auto &row : *this)
            for (auto &cam : row)
                if (cam > max) max = cam;
        return max + 1;
    }

    std::vector<Profile> Profile_list::match(unsigned int area) {
        vector <Profile> matches;
        for (auto &profile : *this)
            if (profile.match(area)) matches.push_back(profile);
        return matches;
    }

    Cameras_associations Cameras_associations::filter(cell_world::Coordinates_list key_points) {
        Cameras_associations filtered;
        for (auto &ca: *this) {
            filtered.emplace_back(ca.filter(key_points));
        }
        return filtered;
    }

    Point::Point(double x, double y) : Location(x, y) {
    }

    Agent_info Frame_detection::to_agent_info() {
        Agent_info ai;
        ai.coordinates = this->detection_coordinates.coordinates;
        ai.location = this->detection_coordinates.detection_location.location;
        ai.theta = this->theta;
        return ai;
    }
}
