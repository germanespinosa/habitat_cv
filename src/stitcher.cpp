#include <stitcher.h>


using namespace std;
using namespace cell_world;
using namespace maze_cv;

Stitcher::Stitcher(const string &stitcher_config_file_path, const string &camera_association_config_file_path) {
    parameters.load(stitcher_config_file_path);
    camera_associations.load(camera_association_config_file_path);
    for (auto &ca : camera_associations){
        vector<cv::Point2f> src_cp;
        vector<cv::Point2f> dst_cp;
        for (auto &a : ca) {
            if (parameters.is_key_point(a.cell_coordinates)) {
                src_cp.push_back(cv::Point2f(a.centroid.x,a.centroid.y));
                dst_cp.push_back(create_point(a.cell_coordinates));
            }
        }
        homographies.push_back(findHomography(src_cp, dst_cp));
    }
    rows = parameters.camera_order.size();
    cols = 0;
    for (auto &r : parameters.camera_order) if (cols < r.size()) cols = r.size();
    cout << "r:" << rows << " c:" << cols << endl;
    crop_size = cv::Size (int(parameters.width/cols), int(parameters.height/rows));
    cout << "w:" << crop_size.width << " h:" << crop_size.height << endl;
    for (unsigned int c = 0; c< camera_associations.size(); c++) {
        auto rect = get_crop_rectangle(c);
        cout << "x:" <<  rect.x << " y:" << rect.y << " w:" << rect.width << " h:" << rect.height << endl;
        crop_rectangles.push_back(rect);
    }
}

/*
Rect srcRect(Point(0, 1), Size(srcMat.cols, 1)); //select the 2nd row
Rect dstRect(Point(3, 5), srcRect.size() ); //destination in (3,5), size same as srcRect

dstRect = dstRect & Rect(Point(0, 0), dstMat.size()); //intersection to avoid out of range
srcRect = Rect(srcRect.tl(), dstRect.size()); //adjust source size same as safe destination
srcMat(srcRect).copyTo(dstMat(dstRect)); //copy from (0,1) to (3,5) max allowed cols

 * */

cv::Point2f Stitcher::create_point(const cell_world::Coordinates &coord) const {
    double center_x = parameters.width / 2;
    double center_y = parameters.height / 2;
    double offset_x = parameters.width / 42 * double(coord.x);
    double offset_y = parameters.height / 22 * double(coord.y);
    return cv::Point2f(center_x-offset_x,center_y+offset_y);
}

cv::Mat Stitcher::get_composite(vector<cv::Mat> &images) const {
    cv::Mat composite(parameters.height,parameters.width,CV_8UC1);
    unsigned int h=0;
    for (auto &image:images){
        cv::Mat warped(parameters.height,parameters.width,CV_8UC1);
        auto &homography = homographies[h];
        cv::warpPerspective(image, warped, homography, warped.size());
        warped(crop_rectangles[h]).copyTo(composite(crop_rectangles[h]));
        h++;
    }
    return composite;
}

cv::Rect Stitcher::get_crop_rectangle(unsigned int c) {
    unsigned int row;
    unsigned int col;
    for (row=0; row< parameters.camera_order.size();row++){
        for (col=0; col<parameters.camera_order[row].size();col++){
            if (parameters.camera_order[row][col] == c)
                return cv::Rect(cv::Point (col * crop_size.width,row * crop_size.height),crop_size);
        }
    }
    return cv::Rect(cv::Point (0,0),crop_size);
}

bool Stitcher_parameters::is_key_point(Coordinates &coord) {
    for (auto &kp: key_points) if (coord == kp) return true;
    return false;
}
