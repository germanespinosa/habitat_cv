#include <habitat_cv/image.h>
#include <habitat_cv/frame_rate.h>
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
        static void start();
        static void stop();
        static cv::Size frame_size;
        static std::vector<Camera *> cameras;
        static std::atomic<bool> running;
        ~Camera();
        Frame_rate frame_rate;
    };
}