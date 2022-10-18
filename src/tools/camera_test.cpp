#include <iostream>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <habitat_cv/camera.h>
#include <habitat_cv/camera_array.h>
#include <habitat_cv/video.h>
#include "opencv2/video.hpp"

using namespace habitat_cv;
using namespace std;
using namespace cv;
int main() {
    Camera_array ca("", 4);
    auto im = ca.capture();
    auto frames_per_second = 90;
    auto frame_size = im[0].size();
    cv::VideoWriter writer;
    cout << frame_size << endl;
    auto b = writer.open("output.mp4", VideoWriter::fourcc('m', 'p', '4', 'v'),frames_per_second, frame_size, false);
    if (!b) return 0;
//    Video v(im[0].size(),Image::gray);
//    v.new_video("here.mp4");
    unsigned int frame = 0;
    while(cv::waitKey(10) != ' '){
        frame ++;
        im = ca.capture();
        for (unsigned int c = 0;c < 4; c++){
            cv::imshow("camera " + to_string(c), im[c]);
        }
//        cv::imwrite("frame_" + to_string(frame) + ".png", im[0]);
        //cout << im[0].size() << endl;
        writer.write(im[0]);
//        v.add_frame(im[0]);
    }
    writer.release();
//    v.close();
}