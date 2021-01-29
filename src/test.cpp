#include <fstream>
#include <stitcher.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

int main(int arc, char **arv){
    Stitcher stitcher("../config/stitcher.config", "../config/associations.config");
    vector<cv::Mat> images;
    for (int i = 0 ;i<4; i++){
        stringstream ss;
        ss << "../images/camera_" << i << ".png";
        cv::Mat greyMat, colorMat;
        colorMat = cv::imread(ss.str());
        cv::cvtColor(colorMat, greyMat, cv::COLOR_BGR2GRAY);
        images.push_back(greyMat);
    }
    cv::Mat composite = stitcher.get_composite(images);
    cv::imshow("composite",composite);
    cv::waitKey(0);
}