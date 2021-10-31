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

    cv::Mat &Composite::get_composite(const Images &images, bool draw_all) {
        for (unsigned int c=0; c< configuration.order.count(); c++){
            cv::warpPerspective(images[c], warped[c], homographies[c], size);
            warped[c](crop_rectangles[c]).copyTo(composite(crop_rectangles[c]));
        }
        cvtColor(composite, rgb_composite, cv::COLOR_GRAY2RGB);
        if (draw_all)
            for (auto coord : valid_coordinates)
                draw_cell(coord);
        return composite;
    }

    Composite::Composite(const Camera_configuration &camera_configuration) :
    configuration(camera_configuration){
        auto composite_space =  Resources::from("space").key("hexagonal").key("composite").get_resource<Space>();
        size = cv::Size(composite_space.transformation.size, composite_space.transformation.size);
        composite = Image(size.height, size.width, Image::Type::gray);
        auto wc =  Resources::from("world_configuration").key("hexagonal").get_resource<World_configuration>();
        auto wi =  Resources::from("world_implementation").key("hexagonal").key("canonical").get_resource<World_implementation>();
        wi.transform(composite_space);
        cells = Polygon_list(wi.cell_locations,wc.cell_shape,wi.cell_transformation);
        world = World(wc, wi);
        map = Map(world.create_cell_group());
        cv::Size crop_size (size.width/configuration.order.cols(), size.height/configuration.order.rows());
        for (unsigned int c=0; c<configuration.order.count(); c++) {
            warped.emplace_back(size.height,size.width,Image::Type::gray);
            auto camera_coordinates = configuration.order.get_camera_coordinates(c);
            cv::Point crop_location (camera_coordinates.x * crop_size.width,
                                     camera_coordinates.y * crop_size.height);
            crop_rectangles.emplace_back( crop_location, crop_size);
            vector<cv::Point2f> src_cp;
            vector<cv::Point2f> dst_cp;
            for (auto &a:configuration.centroids[c]) {
                src_cp.emplace_back(a.centroid.x,a.centroid.y);
                dst_cp.emplace_back(get_point(a.cell_coordinates));
            }
            homographies.push_back(findHomography(src_cp, dst_cp));
        }
    }

    Location Composite::get_location(const Location &point, unsigned int camera) {
        vector<cv::Point2f> src;
        src.push_back(to_point(point));
        vector<cv::Point2f> dst;
        cv::perspectiveTransform(src, dst, homographies[camera]);
        Location p;
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

    void Composite::draw_cell(const cell_world::Coordinates &coordinates, const cv::Scalar color) {
        auto points = get_hexagon(coordinates);
        cv::polylines(rgb_composite,points,true,color,1);
    }

    cell_world::Coordinates Composite::get_coordinates(const cell_world::Location & point) {
        auto cell_id = map.cells.find(point);
        return map.cells[cell_id].coordinates;
    }

    void Composite::draw_circle(const Location &center, int radius, const cv::Scalar color) {
        cv::Point new_center (center.x, flip_y(center.y));
        cv::circle(rgb_composite, new_center, radius, color);
    }

    void Composite::draw_arrow(const Location &center, double theta, const cv::Scalar color, double length) {
        cv::Point2f end;
        end.x = center.x + cos(theta) * world.cell_transformation.size * length;
        end.y = flip_y(center.y + sin(theta) * world.cell_transformation.size * length);
        auto new_center = to_point(center);
        new_center.y = flip_y(new_center.y);
        arrow( rgb_composite, new_center,  end, color, 10);
    }

    float Composite::flip_y(double y) const{
        return (float)size.height - y;
    }

    cv::Mat &Composite::get_rgb() {
        return rgb_composite;
    }

}
