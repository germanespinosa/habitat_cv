#include <composite.h>
using namespace std;

namespace habitat_cv {
    cv::Point Composite::get_point(const cell_world::Coordinates &coord) const {
        double center_x = size.width / 2;
        double center_y = size.height / 2;
        double offset_x = size.width / 40 * double(coord.x);
        double offset_y = size.height / 22 * double(coord.y);
        return cv::Point2f(center_x-offset_x,center_y+offset_y);
    }

    cv::Mat &Composite::get_composite(const std::vector<cv::Mat> &images) {
        for (unsigned int c=0; c<camera_order.count(); c++){
            cv::warpPerspective(images[c], warped[c], homographies[c], size);
            warped[c](crop_rectangles[c]).copyTo(composite(crop_rectangles[c]));
        }
        return composite;
    }

    Composite::Composite(const cv::Size size, const Camera_order &camera_order, const Cameras_associations &key_points) :
    size(size),
    composite(size.height,size.width,CV_8UC1),
    camera_order(camera_order){
        cv::Size crop_size (size.width/camera_order.cols(), size.height/camera_order.rows());
        for (unsigned int c=0; c<camera_order.count(); c++) {
            warped.emplace_back(size.height,size.width,CV_8UC1);
            auto camera_coordinates = camera_order.get_camera_coordinates(c);
            cv::Point crop_location (camera_coordinates.x * crop_size.width,
                                     camera_coordinates.y * crop_size.height);
            crop_rectangles.emplace_back( crop_location, crop_size);
            vector<cv::Point2f> src_cp;
            vector<cv::Point2f> dst_cp;
            for (auto &a : key_points[c]) {
                    src_cp.emplace_back(a.centroid.x,a.centroid.y);
                    dst_cp.emplace_back(get_point(a.cell_coordinates));
            }
            homographies.push_back(findHomography(src_cp, dst_cp));
        }
    }
}
