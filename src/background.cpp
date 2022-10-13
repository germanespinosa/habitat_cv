#include <cell_world.h>
#include <habitat_cv/background.h>

using namespace habitat_cv;
using namespace cell_world;

namespace habitat_cv {
    bool Background::update(habitat_cv::Image new_composite) {
        new_composite.save(path, "composite.png");
        return load();
    }

    bool Background::load() {
        if (file_exists(path, {"composite.png"})) {
            composite = Image::read(path, "composite.png").to_gray();
            return true;
        }
        return false;
    }

    void Background::set_path(const std::string &new_path) {
        path = new_path;
    }
}