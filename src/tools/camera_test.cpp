#include <xcliball.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <iostream>
#include <unistd.h>
#include <cell_world.h>

using namespace std;
using namespace std::this_thread;
using namespace std::chrono;
using namespace cell_world;

atomic<bool> running = true;
vector<float> frame_rates(4,0);

void camera_capture(int camera, int rows, int cols){
    int grabber_bit_map = 1 << camera;
    cout << "starting grabber "<< camera << endl;
    pxd_goLivePair(grabber_bit_map, 1, 2);
    auto buffer = cv::Mat(rows, cols, CV_8UC1);
    auto data = buffer.data;
    auto size = rows * cols;
    float ms = 0;
    long prev = -1;
    long curr = -1;
    int frames = 0;
    time_point<high_resolution_clock> start_time = high_resolution_clock::now();
    while (running) {
        while (prev == curr && running){
            usleep(2000);
            curr = pxd_capturedBuffer(grabber_bit_map);
        }
        frames ++;
        prev = curr;
        pxd_readuchar(grabber_bit_map, prev, 0, 0, cols, rows, data, size, "Grey");
        curr = pxd_capturedBuffer(grabber_bit_map);
        ms = float(duration<double, std::milli>(high_resolution_clock::now() - start_time).count());
        frame_rates[camera] = float(frames) / ms * 1000;
    }
}

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    json_cpp::Json_date jd;
    date::sys_time<std::chrono::milliseconds> &d = jd;
    d = round<milliseconds>(system_clock::now()) - 5h;
    cout << jd << endl;
    exit(0);
    //list of cameras to test
    mutex mtx;
    for (int e = 0; e < 2000; e++) {
        Experiment experiment;
        cout << "Experiment " << e << endl;
        for (float r=0; r<rand()*10000; r++ ) {
            auto &episode = experiment.episodes.emplace_back();
            for (int i = 0; i < 100000; i++) {
                thread ([&episode, &mtx](){
                    Step step;
                    mtx.lock();
                    episode.trajectories.push_back(step);
                    mtx.unlock();
                }).detach();
                thread ([&episode, &mtx](){
                    Step step;
                    mtx.lock();
                    episode.trajectories.push_back(step);
                    mtx.unlock();
                }).detach();
                sleep_for(std::chrono::milliseconds(50));
            }
        }
    }
    exit(0);

    vector<int> active_cameras = {0,1,2,3};
    vector<thread> capture_threads;
    pxd_PIXCIopen("", "", "/usr/local/xcap/settings/xcvidset.fmt");
    auto rows = pxd_imageYdim();
    auto cols = pxd_imageXdim();
    auto size = rows * cols;
    cout << "testing cameras: ";
    for (auto camera:active_cameras) {
        cout << camera << " ";
    }
    cout << endl;
    cout << "rows : "<< rows << endl;
    cout << "cols : "<< cols << endl;
    cout << "image size : "<< size << " bytes" << endl;
    cout << "test starting" << endl;
    for (auto camera:active_cameras){
        frame_rates.emplace_back();
        capture_threads.emplace_back(camera_capture, camera, 0, 0);
    }
    for (unsigned int t = 0;t < 10; t ++){
        sleep_for(seconds(1));
        cout << t + 1 << ": ";
        for (auto &camera: active_cameras){
            cout << (frame_rates[camera]) << " ";
        }
        cout << endl;
    }
    running = false;
    for (auto &capture_thread: capture_threads){
        capture_thread.join();
    }
}