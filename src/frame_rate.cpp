#include<habitat_cv/frame_rate.h>

using namespace std;
namespace habitat_cv {
    Frame_rate::Frame_rate(double filter) :
            last_frame(chrono::high_resolution_clock::now()),
            filter(filter) {
    }

    void Frame_rate::new_frame() {
        auto new_frame = chrono::high_resolution_clock::now();
        auto ms = std::chrono::duration<double, std::milli>(new_frame - last_frame).count();
        last_frame = new_frame;
        current_fps = 1000.0 / double(ms);
        average_fps = (current_fps + (average_fps * (double)frame_counter)) / (double)(frame_counter + 1);
        frame_counter++;
        filtered_fps = filtered_fps * (1 - filter) + current_fps * filter;
    }
}