#include<habitat_cv/camera.h>
#include <xcliball.h>
#include <csignal>
#define SIGNAL_BASE 60
using namespace std;

namespace habitat_cv{

    cv::Size Camera::frame_size;
    std::vector<Camera *> Camera::cameras;
    std::atomic<bool> Camera::running{false};

    int frame_diff(int current, int destination, int buffer_size){
        if (destination>current) return destination-current;
        return destination + buffer_size - current;
    }

    void new_frame(int signal) {
        if (!Camera::running) return;
        int camera_number = signal - SIGNAL_BASE;        //capture_thread = std::thread(capture_process,this);
        auto camera = Camera::cameras[camera_number];
        int size = Camera::frame_size.height * Camera::frame_size.width;
        int destination = (camera->current_frame + 1) % (int)camera->buffer.size();
        if (frame_diff(camera->current_frame, destination, (int)camera->buffer.size()) > 1) return;
        camera->frame_rate.new_frame();
        pxd_readuchar(camera->grabber_bit_map, 1, 0, 0, -1, -1, camera->buffer[destination].data, size, "Grey");
        camera->buffer[destination].time_stamp.reset();
        camera->current_frame = destination;
    }

    Camera::Camera(int camera_number, int buffer_size) : grabber_bit_map(1 << camera_number){
        for (int i = 0; i < buffer_size; i++) {
            buffer.emplace_back(frame_size.height, frame_size.width, Image::Type::gray);
        }
        Camera::cameras.push_back(this);
        pxd_goLive(grabber_bit_map,1);
        signal (SIGNAL_BASE + camera_number, new_frame);
        pxd_eventCapturedFieldCreate(grabber_bit_map, SIGNAL_BASE + camera_number,NULL );
    }

    void Camera::init(const std::string &config_file) {
        pxd_PIXCIopen("-DM 0xF", "", config_file.c_str()); //config_file.c_str());
        frame_size = {pxd_imageXdim(), pxd_imageYdim()};
    }

    Image &Camera::get_current_frame() {
        return buffer[current_frame];
    }

    Camera::~Camera() {
        pxd_goUnLive(grabber_bit_map);
    }

    void Camera::close() {
        pxd_PIXCIclose();
        Camera::cameras.clear();
    }

    Camera::Camera(int grabber_bit_map):Camera(grabber_bit_map, 5) {

    }

    void Camera::start() {
        Camera::running = true;
    }

    void Camera::stop() {
        Camera::running = false;
        this_thread::sleep_for(1000ms);
    }
}