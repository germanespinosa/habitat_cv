#include <fstream>
#include <stitcher.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <composite.h>
using namespace std;
using namespace maze_cv;
using namespace habitat_cv;

int main(int arc, char **arv){
    Cameras_associations associations;
    cell_world::Coordinates_list  key_points;
    associations.load("../config/associations.config");
    key_points.load("../config/key_points.config");
    cv::Size size (980,800);
    Camera_order co;
    co.load("../config/camera_order.config");
    Composite composite(size, co, associations.filter(key_points));
//    Stitcher stitcher("../config/stitcher.config", "../config/associations.config");
    vector<cv::Mat> images;
    for (int i = 0 ;i<4; i++){
        stringstream ss;
        ss << "../images/camera_" << i << ".png";
        cv::Mat greyMat, colorMat;
        colorMat = cv::imread(ss.str());
        cv::cvtColor(colorMat, greyMat, cv::COLOR_BGR2GRAY);
        images.push_back(greyMat);
    }
    cv::Mat comp = composite.get_composite(images);
    cv::imshow("composite",comp);
    cv::waitKey(0);
}