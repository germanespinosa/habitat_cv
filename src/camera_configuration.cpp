#include <habitat_cv/camera_configuration.h>

using namespace cell_world;

namespace habitat_cv {

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
        for (unsigned int r = 0; r < rows(); r++)
            for (unsigned int c = 0; c < (*this).size(); c++)
                if ((*this)[r][c] == camera) return Coordinates{(int)c, (int)r};
        return Coordinates{-1, -1};
    }

    unsigned int Camera_order::count() const {
        unsigned int max = 0;
        for (auto &row : *this)
            for (auto &cam : row)
                if (cam > max) max = cam;
        return max + 1;
    }

}