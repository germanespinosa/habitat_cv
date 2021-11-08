#include <habitat_cv/video.h>

namespace habitat_cv {

    Video::Video(const std::string &file_name, const cv::Size &size, Image::Type type):
    type(type),
    fourcc(cv::VideoWriter::fourcc('m', 'p','4','v')),
//    fourcc(type == Image::Type::rgb ?
//        cv::VideoWriter::fourcc('m', 'p','4','v'):
//        cv::VideoWriter::fourcc('X', 'V','I','V')),
    frame_count(0),
    writer(file_name, fourcc, 20, size, type == Image::Type::rgb){
    }

    void Video::add_frame(const Image &frame) {
        frame_count++;
        writer.write(frame);
    }

    Video::~Video() {
        writer.release();
    }
}
