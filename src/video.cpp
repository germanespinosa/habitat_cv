#include <habitat_cv/video.h>

namespace habitat_cv {

    Video::Video(const std::string &file_name, const cv::Size &size, const std::string &codec):
    fourcc(cv::VideoWriter::fourcc(codec[0], codec[1], codec[2], codec[3])),
    writer(file_name, fourcc, 30, size, true){

    }

    void Video::add_frame(const Image &frame) {
        writer.write(frame);
    }

    Video::~Video() {
        writer.release();
    }
}
