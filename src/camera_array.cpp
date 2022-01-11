#include <habitat_cv/camera_array.h>
#include <iostream>
#include <xcliball.h>

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

    void Camera_array::capture() {
//        vector<thread> threads;
        for (unsigned int c = 0; c < camera_count; c++) {
            uchar *data=images[c].data;
//            threads.push_back(thread([c](uchar *data, unsigned int frame_size) {
                int grabber_bit_map = 1 << c; // frame grabber identifier is 4 bits with a 1 on the device number.
                pxd_readuchar(grabber_bit_map, 1, 0, 0, -1, -1, data, frame_size, "Grey");
//            }, data, frame_size));
        }
//        for (auto &t:threads) t.join();
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
        for (unsigned int c = 0; c < camera_count; c++) {
            images.emplace_back(size.height, size.width, Image::Type::gray);
        }
        frame_size = size.width * size.height;
    }

    void Camera_array::close() {
        pxd_PIXCIclose();
    }
}

