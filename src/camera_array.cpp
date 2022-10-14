#include <habitat_cv/camera_array.h>
#include <iostream>
using namespace std;
using namespace habitat_cv;

namespace habitat_cv {

    Camera_array::Camera_array(const std::string &config_file_path, unsigned int camera_count) :
            config_file_path(config_file_path),camera_count(camera_count){
        Camera::init(config_file_path);
        for (unsigned int i=0;i<camera_count;i++){
            cameras.emplace_back(new Camera(i, 5));
        }
        Camera::start();
    }

    Camera_array::~Camera_array() {
        Camera::stop();
        cameras.clear();
        Camera::close();
    }

    Images Camera_array::capture() {
        Images images;
        int i=0;
        for (auto &camera:cameras) {
            auto &image = images.emplace_back(camera->get_current_frame());
            image.file_name = "camera_" + to_string(i++) + ".png";
        }
        return images;
    }

    void Camera_array::reset() {
        Camera::stop();
        for (auto &camera:cameras){
            delete camera;
        }
        cameras.clear();
        Camera::close();
        Camera::init(config_file_path);
        for (unsigned int i=0;i<camera_count;i++){
            cameras.emplace_back(new Camera(i, 5));
        }
        Camera::start();
    }
}

