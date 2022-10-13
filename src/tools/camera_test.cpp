#include <iostream>
#include <xcliball.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <habitat_cv/camera.h>

using namespace habitat_cv;
using namespace std;
int main() {
    Camera::init("/usr/local/xcap/settings/xcvidset.fmt");
    vector<Camera *> cameras;
    for (int i=0;i<4;i++) {
        cameras.emplace_back(new Camera(i,10));
    }
    while (cv::waitKey(1) != ' '){
        for (int i=0;i<4;i++){
            cv::imshow(("camera_" + to_string(i)).c_str(), cameras[i]->get_current_frame());
        }
    }
    Camera::close();
}