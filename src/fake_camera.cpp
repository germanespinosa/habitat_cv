#include<habitat_cv/camera.h>

using namespace std;

namespace habitat_cv{

    cv::Size Camera::frame_size;

    Camera::Camera(int camera_number, int ) : grabber_bit_map(0){
        buffer.emplace_back(Image::read("../fake_camera","camera_" + to_string(camera_number) + ".png"));
    }

    void Camera::init(const std::string &) {
    }

    Image &Camera::get_current_frame() {
        return buffer[0];
    }

    Camera::~Camera() {
    }

    void Camera::close() {
    }

    Camera::Camera(int grabber_bit_map):Camera(grabber_bit_map, 5) {

    }
}