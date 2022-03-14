#include <habitat_cv/camera_array.h>
#include <iostream>
#include <xcliball.h>
#include <thread>
using namespace std;
using namespace habitat_cv;

namespace habitat_cv {

    Camera_array::Camera_array(const std::string &config_file_path, unsigned int camera_count) :
            camera_count(camera_count), config_file(config_file_path) {
        open();
    }

    Camera_array::~Camera_array() {
        close();
    }

    Images &Camera_array::capture() {
        while (pxd_capturedBuffer(0x1) == last);  // nothing new captured yet
        last = pxd_capturedBuffer(0x1);
        for (unsigned int c = 0; c < this->camera_count; c++) {
            uchar *data = buffers[c].data;
            int grabber_bit_map = 1 << c; // frame grabber identifier is 4 bits with a 1 on the device number.
            pxd_readuchar(grabber_bit_map, last, 0, 0, -1, -1, data, frame_size, "Grey");
        }
        return buffers;
    }

    void Camera_array::reset() {
        close();
        open();
    }

    void Camera_array::open() {
        pxd_PIXCIopen("", "", config_file.c_str());
        //        if (pxd_goLive(15, 1)) {
        auto err = pxd_goLivePair(7, 1, 2);
        if (err) {
            cerr << "Failed to initialize frame grabbers" << endl;
            exit(1);
        }
        cv::Size size = {pxd_imageXdim(), pxd_imageYdim()};

        for (unsigned int c = 0; c < camera_count; c++) {
            auto &image = buffers.emplace_back(size.height, size.width, Image::Type::gray);
            image.file_name = "camera_" + to_string(c) + ".png";
        }
        frame_size = size.width * size.height;
    }

    void Camera_array::close() {
        pxd_PIXCIclose();
    }
}

