#include <habitat_cv/composite.h>
#include <cell_world.h>
#include <utility>
#include <performance.h>

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

        composite = Image(size.height, size.width, Image::Type::gray);
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

#ifdef USE_CUDA
        gpu_mask_detection.upload(mask_image.threshold(0));
#endif

        auto t = world.space.transformation;
        t.size *= 1.05;
        Polygon video_polygon(world.space.center, world.space.shape, t);
        mask_image.polygon(video_polygon,{255},true);


        mask_video = mask_image.threshold(0);

#ifdef USE_CUDA
        gpu_mask_video.upload(mask_image.threshold(0));
#endif

        map = Map(world.create_cell_group());

        //creates the crop rectangles for each camera
        cv::Size crop_size (size.width / configuration.order.cols(), size.height / configuration.order.rows());
        for (unsigned int c = 0; c < configuration.order.count(); c++) {
            warped.emplace_back(size.height, size.width,Image::Type::gray);
            warped_detection.emplace_back(size.height, size.width,Image::Type::gray);
            warped_video.emplace_back(size.height, size.width,Image::Type::gray);

#ifdef USE_CUDA
            gpu_threads.emplace_back();
            gpu_streams.emplace_back(cudaStreamNonBlocking);
            gpu_raw.emplace_back(size.height, size.width, CV_8UC1);
            gpu_warped.emplace_back(size.height, size.width, CV_8UC1);
            gpu_warped_detection.emplace_back(size.height, size.width, CV_8UC1);
            gpu_warped_video.emplace_back(size.height, size.width, CV_8UC1);
            gpu_composite.upload(composite);
#endif
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
#ifdef USE_CUDA
        for (unsigned int c = 0; c < configuration.order.count(); c++){
            gpu_threads[c] = thread( [this, images] (unsigned int c) {
                auto &stream = gpu_streams[c];
                gpu_raw[c].upload(images[c], stream);
                cv::cuda::warpPerspective(gpu_raw[c], gpu_warped[c], homographies[c], size, cv::INTER_LINEAR,
                                          cv::BORDER_CONSTANT, cv::Scalar(), stream);
                cv::cuda::bitwise_and(gpu_warped[c], gpu_mask_detection, gpu_warped_detection[c], cv::noArray(),stream);
                gpu_warped[c](crop_rectangles[c]).copyTo(gpu_composite(crop_rectangles[c]), stream);
            }, c);
        }
        for (unsigned int c = 0; c < configuration.order.count(); c++) {
            gpu_streams[c].waitForCompletion();
            if (gpu_threads[c].joinable()) gpu_threads[c].join();
        }
        cv::cuda::bitwise_and(gpu_composite, gpu_mask_video, gpu_composite_video, cv::noArray());
        cv::cuda::bitwise_and(gpu_composite, gpu_mask_detection, gpu_composite_detection, cv::noArray());
        gpu_composite_video.download(composite);
        gpu_composite_detection.download(composite_detection);
        composite_video = composite.to_rgb();
        return composite_video;
#else
        for (unsigned int c = 0; c < configuration.order.count(); c++){
            cv::warpPerspective(images[c], warped[c], homographies[c], size);
            warped[c](crop_rectangles[c]).copyTo(composite(crop_rectangles[c]));
        }
        composite_video = composite.mask(mask_video).to_rgb();
        composite_detection = composite.mask(mask_detection);
        return composite_video;
#endif
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

    Image &Composite::get_camera(unsigned int camera_index) {
#ifdef USE_CUDA
        gpu_warped[camera_index].download(warped[camera_index]);
#endif
        warped_detection[camera_index] = warped[camera_index].mask(mask_detection);
        return warped_detection[camera_index];
    }
}
