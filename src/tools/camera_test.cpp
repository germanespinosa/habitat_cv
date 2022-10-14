#include <iostream>
#include <xcliball.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <habitat_cv/camera.h>

using namespace habitat_cv;
using namespace std;

cv::Rect_<int> f(cv::Size rs, cv::Size zs, cv::Point2f point, cv::Point2f &oftl){
    cv::Point2f offset(zs.width / 2,zs.height / 2);
    auto tl = point - offset;
    auto br = point + offset;
    oftl=cv::Point2f(0,0);
    cv::Size ofbr (0,0);
    if (tl.x<0) {
        oftl.x = tl.x;
        ofbr.width = -tl.x;
    }
    if (tl.y<0) {
        oftl.y = tl.y;
        ofbr.height = -tl.y;
    }
    if (br.x > rs.width) {
        ofbr.width = br.x - rs.width;
    }
    if (br.y > rs.height) {
        ofbr.height = br.y - rs.height;
    }
    return cv::Rect_<int>(tl-oftl, zs-ofbr);
}

int main() {
    cv::Point2f p(0,0);
    auto r1 = f({100,100},{30,30},{90,10}, p);
    cout << r1.x << " " << r1.y << " " << r1.width << " " << r1.height << " " << p.x << " " << p.y;
}