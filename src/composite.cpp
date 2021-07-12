#include <habitat_cv/composite.h>
#include <math.h>
using namespace std;

namespace habitat_cv {
    cv::Point Composite::get_point(const cell_world::Coordinates &coord) const {
        if (is_valid(coord))
            return cell_centroids[coord.x+20][coord.y+10];
        return cv::Point2f(0,0);
    }

    cv::Mat &Composite::get_composite(const std::vector<cv::Mat> &images, bool draw_all) {
        for (unsigned int c=0; c<camera_order.count(); c++){
            cv::warpPerspective(images[c], warped[c], homographies[c], size);
            warped[c](crop_rectangles[c]).copyTo(composite(crop_rectangles[c]));
        }
        if (draw_all)
            for (auto coord : valid_coordinates)
                    draw_cell(composite, coord);
        return composite;
    }

    Composite::Composite(const cv::Size size, const Camera_order &camera_order, const Cameras_associations &key_points) :
    size(size),
    composite(size.height,size.width,CV_8UC1),
    camera_order(camera_order){
        double center_x = size.width / 2;
        double center_y = size.height / 2;
        for (int x = -20; x <= 20; x++){
            cell_centroids.emplace_back();
            for (int y = -10; y <= 10; y++){
                auto coord = cell_world::Coordinates{x, y};
                double offset_x = size.width / 42 * double(x);
                double offset_y = size.height / 22 * double(y);
                cell_centroids.back().push_back(cv::Point2f(center_x+offset_x,center_y-offset_y));
                if (is_valid(coord)) {
                    valid_coordinates.push_back(coord);
                }
            }
        }
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

    Point Composite::get_location(Point point, unsigned int camera) {
        vector<cv::Point2f> src;
        src.push_back(point.to<cv::Point2f>());
        vector<cv::Point2f> dst;
        cv::perspectiveTransform(src, dst, homographies[camera]);
        Point p;
        p.x = dst[0].x / (double)size.width;
        p.y = dst[0].y / (double)size.height;
        return p;
    }

    std::vector<cv::Point> Composite::get_hexagon(const cell_world::Coordinates &coordinates) const {
        auto center = get_point(coordinates);
        double radius = size.width * 0.027492869961410742;
        vector<cv::Point> points;
        float c = M_PI / 3;
        for (float i=0;i<6;i++){
            float x = sin(i*c) * radius + center.x;
            float y = cos(i*c) * radius + center.y;
            points.emplace_back(x,y);
        }
        return points;
    }

    void Composite::draw_cell(cv::Mat &img, const cell_world::Coordinates &coordinates, const cv::Scalar color) const{
        auto points = get_hexagon(coordinates);
        cv::polylines(img,points,true,color,1);
//        auto c = to_string(coordinates.x);
//        cv::putText(composite,
//                    c.c_str(),
//                    get_point(coordinates), // Coordinates
//                    cv::FONT_HERSHEY_DUPLEX, // Font
//                    .5, // Scale. 2.0 = 2x bigger
//                    cv::Scalar(255,255,255), // BGR Color
//                    1 // Line Thickness (Optional)
//        );
    }

    cell_world::Coordinates Composite::get_coordinates(cv::Point point) {
        cell_world::Coordinates best;
        float best_distance = -1;
        for (auto coord : valid_coordinates)
        {
            auto centroid = get_point(coord);
            auto distance = cv::norm(point-centroid);
            if (best_distance == -1 || distance < best_distance ){
                best_distance = distance;
                best = coord;
            }
        }
        return best;
    }

    void Composite::draw_circle(cv::Mat &img, cv::Point &center, int radius, const cv::Scalar color) const {
        cv::circle(img, center, radius, color);
    }

    bool Composite::is_valid(const cell_world::Coordinates &coord) const {
        auto check = abs(coord.x) + abs(coord.y);
        return check <= 20 && check % 2 == 0;
    }

    void Composite::draw_arrow(cv::Mat &img, cv::Point &center, double theta, const cv::Scalar color) const {
        int thickness = 2;
        int lineType = cv::LINE_8;
        cv::Point end;
        end.x = center.x + cos(theta) * 50;
        end.y = center.y + sin(theta) * 50;
        cv::line( img, center,  end, color, thickness, lineType );
    }

}
