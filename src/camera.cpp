#include<habitat_cv/camera.h>
#include <xcliball.h>

using namespace std;

namespace habitat_cv{

    cv::Size Camera::frame_size;


    void capture_process (Camera *camera){
        camera->running = true;
        long prev = -1;
        int size = Camera::frame_size.height * Camera::frame_size.width;
        pxd_goLivePair(camera->grabber_bit_map,1,2);
        while (camera->running){
            int destination = (camera->current_frame + 1) % (int)camera->buffer.size();
            while(pxd_capturedBuffer(camera->grabber_bit_map)==prev && camera->running );
            prev = pxd_capturedBuffer(camera->grabber_bit_map);
            camera->frame_rate.new_frame();
//            thread([camera](long prev, int destination, int size){
//4177920
                pxd_readuchar(camera->grabber_bit_map, prev, 0, 0, -1, -1, camera->buffer[destination].data, size, "Grey");
//            }, prev, destination, size).detach();
//            t.wait(.02);
            camera->buffer[destination].time_stamp.reset();
            camera->current_frame = destination;
        }
    }

    Camera::Camera(int grabber_bit_map, int buffer_size) : grabber_bit_map(grabber_bit_map){
        for (int i=0; i<buffer_size; i++) {
            buffer.emplace_back(frame_size.height, frame_size.width, Image::Type::gray);
        }
        current_frame = -1;
        capture_thread = std::thread(capture_process,this);
        while (current_frame == -1);
    }

    void Camera::init(const std::string &config_file) {
        pxd_PIXCIopen("", "", config_file.c_str());
        frame_size = {pxd_imageXdim(), pxd_imageYdim()};
    }

    Image &Camera::get_current_frame() {
        return buffer[current_frame];
    }

    Camera::~Camera() {
        running = false;
        if (capture_thread.joinable()) capture_thread.join();
    }

    void Camera::close() {
        pxd_PIXCIclose();
    }

    Camera::Camera(int grabber_bit_map):Camera(grabber_bit_map, 5) {

    }
}