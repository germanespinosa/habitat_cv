#include <habitat_cv/composite.h>
#include <cell_world.h>

using namespace json_cpp;
using namespace std;
using namespace cell_world;

namespace habitat_cv {
    Composite::Composite(const Camera_configuration &camera_configuration) :
    configuration(camera_configuration){
        auto wc = Resources::from("world_configuration")
                .key("hexagonal")
                .get_resource<World_configuration>();
        auto wi = Resources::from("world_implementation")
                .key("hexagonal")
                .key("cv")
                .get_resource<World_implementation>();

        size = cv::Size(wi.space.transformation.size, wi.space.transformation.size);
        size_large = cv::Size(wi.space.transformation.size * 2, wi.space.transformation.size * 2);

        composite = Image(size.height, size.width, Image::Type::gray);
        composite_large = Image(size.height * 2, size.width * 2, Image::Type::gray);

        cells = Polygon_list(wi.cell_locations, wc.cell_shape, wi.cell_transformation);

        world = World(wc, wi);

        // generate mask
        Image mask_image(size.height, size.width, Image::Type::gray);
        mask_image.clear();
        auto t = world.space.transformation;
        t.size *= 1.05;
        Polygon habitat_polygon(world.space.center, world.space.shape, t);
        mask_image.polygon(habitat_polygon,{255},true);
        mask = mask_image.threshold(0);

        //generates a large implementation to improve definition
        auto wi_large = wi;
        cout << "small:" << wi << endl;
        for (auto &location : wi_large.cell_locations) location = location * 2;
        wi_large.cell_transformation.size *= 2;
        wi_large.space.transformation.size *= 2;
        wi_large.space.center = wi_large.space.center * 2;
        cout << "large:" << wi_large << endl;

        cells_large = Polygon_list(wi_large.cell_locations, wc.cell_shape, wi_large.cell_transformation);
        world_large = World(wc, wi_large);

        // generate large mask
        Image mask_image_large(size_large.height, size_large.width, Image::Type::gray);
        mask_image_large.clear();
        auto t_large = world_large.space.transformation;
        t_large.size *= 1.05;
        Polygon habitat_polygon_large(world_large.space.center, world_large.space.shape, t_large);
        mask_image_large.polygon(habitat_polygon_large,{255},true);
        mask_large = mask_image_large.threshold(0);


        map = Map(world.create_cell_group());
        map_large = Map(world_large.create_cell_group());


        //creates the crop rectangles for each camera
        cv::Size crop_size (size_large.width / configuration.order.cols(), size_large.height / configuration.order.rows());
        for (unsigned int c = 0; c < configuration.order.count(); c++) {
            warped.emplace_back(size_large.height, size_large.width,Image::Type::gray);
            auto camera_coordinates = configuration.order.get_camera_coordinates(c);
            cv::Point crop_location (camera_coordinates.x * crop_size.width,
                                     camera_coordinates.y * crop_size.height);
            crop_rectangles.emplace_back(crop_location, crop_size);
            vector<cv::Point2f> src_cp;
            vector<cv::Point2f> dst_cp;
            for (auto &a:configuration.centroids[c]) {
                src_cp.emplace_back(a.centroid.x,a.centroid.y);
                dst_cp.emplace_back(composite_large.get_point(map.cells[map_large.find(a.cell_coordinates)].location));
            }
            homographies.push_back(findHomography(src_cp, dst_cp));
        }
    }

    cell_world::Coordinates Composite::get_coordinates(const cell_world::Location & point) {
        auto cell_id = map.cells.find(point);
        return map.cells[cell_id].coordinates;
    }

    Image &Composite::get_composite(const Images &images) {

        for (unsigned int c = 0; c < configuration.order.count(); c++){
            Image w;
            w.type = images[c].type;
            cv::warpPerspective(images[c], w, homographies[c], size_large);
            warped[c] = w.mask(mask_large);
            warped[c](crop_rectangles[c]).copyTo(composite_large(crop_rectangles[c]));
        }
        resize(composite_large, composite, size, cv::INTER_LINEAR);
        return composite;
    }

    cell_world::Polygon &Composite::get_polygon(const Coordinates &coordinates) {
        return cells[map.find(coordinates)];
    }
}
