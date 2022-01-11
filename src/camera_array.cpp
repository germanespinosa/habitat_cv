#include <habitat_cv/camera_array.h>
#include <iostream>
#include <xcliball.h>
#include <thread>
using namespace std;
using namespace habitat_cv;

namespace habitat_cv {

    Camera_array::Camera_array(const std::string &config_file_path, unsigned int camera_count) :
            camera_count(camera_count), config_file(config_file_path), active_buffer(0), ready_buffer(0) {
        open();
        running = false;
        unfinished_capture = thread([this](){
            running = true;
            while(running){
                while(active_buffer != ready_buffer);
                for (unsigned int c = 0; c < this->camera_count; c++) {
                    uchar *data = buffers[active_buffer][c].data;
                    int grabber_bit_map = 1 << c; // frame grabber identifier is 4 bits with a 1 on the device number.
                    pxd_readuchar(grabber_bit_map, 1, 0, 0, -1, -1, data, frame_size, "Grey");
                }
                ready_buffer= (ready_buffer + 1) % 2;
            }
        });
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
        if (pxd_goLive(15, 1)) {
            cerr << "Failed to initialize frame grabbers" << endl;
            exit(1);
        }
        cv::Size size = {pxd_imageXdim(), pxd_imageYdim()};
        for (unsigned int b = 0; b < 2; b++) {
            auto &images=buffers.emplace_back();
            for (unsigned int c = 0; c < camera_count; c++) {
                images.emplace_back(size.height, size.width, Image::Type::gray);
            }
        }
        frame_size = size.width * size.height;
    }

    void Camera_array::close() {
        running = false;
        if (unfinished_capture.joinable()) unfinished_capture.join();
        pxd_PIXCIclose();
    }
}

