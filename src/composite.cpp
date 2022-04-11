#include <habitat_cv/composite.h>
#include <cell_world.h>
#include <thread>

#include <utility>

using namespace json_cpp;
using namespace std;
using namespace cell_world;

namespace habitat_cv {
    Composite::Composite(Camera_configuration camera_configuration) :
    configuration(std::move(camera_configuration)){
        auto wc = Resources::from("world_configuration")
                .key("hexagonal")
                .get_resource<World_configuration>();
        auto wi = Resources::from("world_implementation")
                .key("hexagonal")
                .key("cv")
                .get_resource<World_implementation>();
        size = cv::Size(wi.space.transformation.size, wi.space.transformation.size);

        composite_video = Image(size.height, size.width, Image::Type::gray);
        composite_detection = Image(size.height, size.width, Image::Type::gray);

        cells = Polygon_list(wi.cell_locations, wc.cell_shape, Transformation(wi.cell_transformation.size , wi.cell_transformation.rotation + wi.space.transformation.rotation));

        world = World(wc, wi);

        // generate mask
        Image mask_image(size.height, size.width, Image::Type::gray);
        mask_image.clear();
        Polygon detection_polygon(world.space.center, world.space.shape, world.space.transformation);
        mask_image.polygon(detection_polygon,{255},true);
        mask_detection = mask_image.threshold(0);

        auto t = world.space.transformation;
        t.size *= 1.05;
        Polygon video_polygon(world.space.center, world.space.shape, t);
        mask_image.polygon(video_polygon,{255},true);
        mask_video = mask_image.threshold(0);

        map = Map(world.create_cell_group());

        //creates the crop rectangles for each camera
        cv::Size crop_size (size.width / configuration.order.cols(), size.height / configuration.order.rows());
        for (unsigned int c = 0; c < configuration.order.count(); c++) {
            warped.emplace_back(size.height, size.width,Image::Type::gray);
            warped_detection.emplace_back(size.height, size.width,Image::Type::gray);
            warped_video.emplace_back(size.height, size.width,Image::Type::gray);
            auto camera_coordinates = configuration.order.get_camera_coordinates(c);
            cv::Point crop_location (camera_coordinates.x * crop_size.width,
                                     camera_coordinates.y * crop_size.height);
            crop_rectangles.emplace_back(crop_location, crop_size);
            vector<cv::Point2f> src_cp;
            vector<cv::Point2f> dst_cp;
            for (auto &a:configuration.centroids[c]) {
                src_cp.emplace_back(a.centroid.x,a.centroid.y );
                dst_cp.emplace_back(composite_video.get_point(map.cells[map.find(a.cell_coordinates)].location));
            }
            inverted_homographies.emplace_back(homographies.emplace_back(findHomography(src_cp, dst_cp)).inv());
        }
    }

    cell_world::Coordinates Composite::get_coordinates(const cell_world::Location & point) {
        auto cell_id = map.cells.find(point);
        return map.cells[cell_id].coordinates;
    }

    Image &Composite::get_composite(const Images &images) {
        for (unsigned int c = 0; c < configuration.order.count(); c++){
            cv::warpPerspective(images[c], warped[c], homographies[c], size);
            warped_detection[c] = warped[c].mask(mask_detection);
            warped_video[c] = warped[c].mask(mask_video);
            warped_detection[c](crop_rectangles[c]).copyTo(composite_detection(crop_rectangles[c]));
            warped_video[c](crop_rectangles[c]).copyTo(composite_video(crop_rectangles[c]));
       }
        return composite_video;
    }

    cv::Point2f Composite::get_raw_point(unsigned int camera_index, const cv::Point2f &composite_point){
        cv::Matx33f warp = inverted_homographies[camera_index];
        cv::Point2f warped_point(composite_point.x,composite_point.y);
        cv::Point3f homogeneous = warp * warped_point;
        return {homogeneous.x, homogeneous.y};
    }

    cell_world::Polygon &Composite::get_polygon(const Coordinates &coordinates) {
        return cells[map.find(coordinates)];
    }

    cv::Point2f Composite::get_warped_point(unsigned int camera_index, const cv::Point2f &camera_point) {
        cv::Matx33f warp = homographies[camera_index];
        cv::Point2f homogeneous_point(camera_point.x,camera_point.y);
        cv::Point3f warped = warp * homogeneous_point;
        return {warped.x, warped.y};
    }
}
