#include <habitat_cv/camera_array.h>
#include <iostream>
#include <xcliball.h>
#include <thread>
using namespace std;
using namespace habitat_cv;

namespace habitat_cv {

    Camera_array::Camera_array(const std::string &config_file_path, unsigned int camera_count) :
            camera_count(camera_count), config_file(config_file_path), active_buffer(0), ready_buffer(0) {
        running = false;
        open();
        while (!running);
    }

    Camera_array::~Camera_array() {
        close();
    }

    Images &Camera_array::capture() {
        while(active_buffer==ready_buffer);
        active_buffer=( active_buffer + 1 ) % 2;
        return buffers[ready_buffer];
    }

    void Camera_array::reset() {
        close();
        open();
    }

    void Camera_array::open() {
        pxd_PIXCIopen("", "", config_file.c_str());
        cv::Size size = {pxd_imageXdim(), pxd_imageYdim()};
        for (unsigned int b = 0; b < 2; b++) {
            auto &images=buffers.emplace_back();
            for (unsigned int c = 0; c < camera_count; c++) {
                auto &image = images.emplace_back(size.height, size.width, Image::Type::gray);
                image.file_name = "camera_" + to_string(c) + ".png";
            }
        }
        active_buffer = 0;
        ready_buffer = 0;
        frame_size = size.width * size.height;
        unfinished_capture = thread([this](){
            running = true;
            for (unsigned int c = 0; c < camera_count && running; c++) {
                int grabber_bit_map = 1 << c;
                pxd_goSnap(grabber_bit_map, 1);
            }
            cell_world::Timer t(.02);
            while(running){
                while(active_buffer != 0 && running);
                while (!t.time_out());
                for (unsigned int c = 0; c < camera_count && running; c++){
                    int grabber_bit_map = 1 << c;
                    while(pxd_capturedBuffer(grabber_bit_map)!=1 && running );
                    pxd_readuchar(grabber_bit_map, 1, 0, 0, -1, -1, buffers[0][c].data, frame_size, "Grey");
                    pxd_goSnap(grabber_bit_map, 2);
                }
                t.reset();
                ready_buffer = 1;
                while(active_buffer != 1 && running );
                while (!t.time_out());
                for (unsigned int c = 0; c < camera_count && running; c++){
                    int grabber_bit_map = 1 << c;
                    while(pxd_capturedBuffer(grabber_bit_map)!=2 && running );
                    pxd_readuchar(grabber_bit_map, 2, 0, 0, -1, -1, buffers[1][c].data, frame_size, "Grey");
                    pxd_goSnap(grabber_bit_map, 1);
                }
                t.reset();
                ready_buffer = 0;
            }
        });
    }

    void Camera_array::close() {
        running = false;
        if (unfinished_capture.joinable()) unfinished_capture.join();
        pxd_PIXCIclose();
    }
}

