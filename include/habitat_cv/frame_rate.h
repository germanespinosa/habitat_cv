#pragma once
#include <chrono>

namespace habitat_cv {
    struct Frame_rate {
        Frame_rate(double filter = .1);

        std::chrono::time_point<std::chrono::high_resolution_clock> last_frame;
        unsigned int frame_counter{};
        double average_fps{};
        double filter{};
        double current_fps{};
        double filtered_fps{};

        void new_frame();
    };
}