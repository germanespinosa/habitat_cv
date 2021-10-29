#include <habitat_cv/composite.h>
#include <cell_world.h>
#include <math.h>

using namespace json_cpp;
using namespace std;
using namespace cell_world;

namespace habitat_cv {
    cv::Point Composite::get_point(const cell_world::Coordinates &coord) const {
        auto cell_id = map.find(coord);
        if (cell_id == Not_found) return cv::Point2f(0,0);
        auto location = map.cells[cell_id].location;
        return cv::Point2f(location.x,  flip_y(location.y));
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

    Composite::Composite(const Camera_order &camera_order, const Cameras_associations &key_points) :
    camera_order(camera_order){
        auto composite_space =  Json_create<Space>(Web_resource::from("space").key("hexagonal").key("composite").get());
        size = cv::Size(composite_space.transformation.size, composite_space.transformation.size);
        composite= cv::Mat(size.height,size.width,CV_8UC1);

        auto wc =  Json_create<World_configuration>(Web_resource::from("world_configuration").key("hexagonal").get());
        auto wi =  Json_create<World_implementation>(Web_resource::from("world_implementation").key("hexagonal").key("canonical").get());
        wi.transform(composite_space);
        cells = Polygon_list(wi.cell_locations,wc.cell_shape,wi.cell_transformation);
        world = World(wc, wi);
        map = Map(world.create_cell_group());

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
        auto cell_id = map.find(coordinates);
        auto &polygon = cells[cell_id];

        vector<cv::Point> points;
        float c = M_PI / 3;
        for (auto &vertex:polygon.vertices){
            points.emplace_back(vertex.x,flip_y(vertex.y));
        }
        return points;
    }

    void Composite::draw_cell(cv::Mat &img, const cell_world::Coordinates &coordinates, const cv::Scalar color) const{
        auto points = get_hexagon(coordinates);
        cv::polylines(img,points,true,color,1);
    }

    cell_world::Coordinates Composite::get_coordinates(cv::Point point) {
        auto cell_id = map.cells.find(Location(point.x, point.y));
        return map.cells[cell_id].coordinates;
    }

    void Composite::draw_circle(cv::Mat &img, const cv::Point &center, int radius, const cv::Scalar color) const {
        cv::Point new_center (center.x, flip_y(center.y));
        cv::circle(img, new_center, radius, color);
    }

    void Composite::draw_arrow(cv::Mat &img, const cv::Point &center, double theta, const cv::Scalar color) const {
        int thickness = 2;
        int lineType = cv::LINE_8;
        cv::Point end;
        end.x = center.x + cos(theta) * 50;
        end.y = flip_y(center.y) + sin(theta) * 50;
        auto new_center = center;
        new_center.y = flip_y(new_center.y);
        cv::line( img, new_center,  end, color, thickness, lineType );
    }

    float Composite::flip_y(double y) const{
        return (float)size.height - y;
    }

}
