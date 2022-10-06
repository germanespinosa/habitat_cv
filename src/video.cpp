#include <habitat_cv/video.h>
#include <mutex>

namespace habitat_cv {

    unsigned int fps = 60;

    Video::Video(const cv::Size &size, Image::Type type):
    frame_count(-1),
    size(size),
    type(type),
    fourcc(cv::VideoWriter::fourcc('m', 'p','4','v'))
    {
    }

    bool Video::add_frame(const Image &frame) {
        if (frame.type != type || frame.size() != size) return false;
        write_mutex.lock();
        if (is_open()) {
            frame_count++;
            writer.write(frame);
            write_mutex.unlock();
            return true;

        }
        write_mutex.unlock();
        return false;
    }

    Video::~Video() {
        close();
    }

    bool Video::new_video(const std::string &file_name) {
        writer = cv::VideoWriter(file_name, fourcc, fps, size, type == Image::Type::rgb);
        frame_count = 0;
        return false;
    }

    bool Video::close() {
        write_mutex.lock();
        if (!is_open()) {
            write_mutex.unlock();
            return false;
        }
        writer.release();
        frame_count = -1;
        write_mutex.unlock();
        return true;
    }

    bool Video::is_open() const {
        return frame_count >= 0;
    }

    void Video::set_fps(unsigned int new_fps) {
        fps = new_fps;
    }

    unsigned int Video::get_fps() {
        return fps;
    }
}
