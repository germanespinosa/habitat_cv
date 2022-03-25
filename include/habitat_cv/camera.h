#include <habitat_cv/image.h>
#include <thread>
namespace habitat_cv {
    struct Camera {
        Camera(int, int);
        Camera(int);
        Image &get_current_frame();
        int grabber_bit_map;
        Images buffer;
        std::atomic<int> current_frame;
        static void init(const std::string &config_file);
        static void close();
        static cv::Size frame_size;
        std::atomic<bool> running;
        std::thread capture_thread;
        ~Camera();
    };
}