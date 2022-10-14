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
        composite_video = Image(size.height, size.width, Image::Type::rgb);
        composite_detection = Image(size.height, size.width, Image::Type::gray);

#ifdef USE_CUDA
        gpu_composite_video.upload(composite);
        gpu_composite_detection.upload(composite_detection);
        gpu_composite_video_rgb.upload(composite_detection);
#endif
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
            zoom.emplace_back(zoom_size.height, zoom_size.width,Image::Type::gray);
            warped.emplace_back(size.height, size.width,Image::Type::gray);
            warped_detection.emplace_back(size.height, size.width,Image::Type::gray);
            warped_video.emplace_back(size.height, size.width,Image::Type::gray);

#ifdef USE_CUDA
            gpu_zoom_streams.emplace_back(cudaStreamNonBlocking);
            gpu_zoom.emplace_back(zoom_size.height, zoom_size.width, CV_8UC1);
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
        raw = images;
#ifdef USE_CUDA
        for (unsigned int c = 0; c < configuration.order.count(); c++){
            auto &stream = gpu_streams[c];
            gpu_raw[c].upload(raw[c], stream);
            cv::cuda::warpPerspective(gpu_raw[c], gpu_warped[c], homographies[c], size, cv::INTER_LINEAR,
                                      cv::BORDER_CONSTANT, cv::Scalar(), stream);
            cv::cuda::bitwise_and(gpu_warped[c], gpu_mask_detection, gpu_warped_detection[c], cv::noArray(),stream);
            gpu_warped[c](crop_rectangles[c]).copyTo(gpu_composite(crop_rectangles[c]), stream);
        }
        for (unsigned int c = 0; c < configuration.order.count(); c++) {
            gpu_streams[c].waitForCompletion();
        }
        cv::cuda::bitwise_and(gpu_composite, gpu_mask_video, gpu_composite_video, cv::noArray(), gpu_video_stream);
        gpu_composite_video.download(composite, gpu_video_stream);
        cv::cuda::bitwise_and(gpu_composite, gpu_mask_detection, gpu_composite_detection, cv::noArray());
        gpu_composite_detection.download(composite_detection);
        return composite_detection;
#else
        for (unsigned int c = 0; c < configuration.order.count(); c++){
            cv::warpPerspective(raw[c], warped[c], homographies[c], size);
            warped[c](crop_rectangles[c]).copyTo(composite(crop_rectangles[c]));
        }
        composite = composite.mask(mask_video);
        composite_detection = composite.mask(mask_detection);
        return composite_detection;
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

    Images &Composite::get_zoom() {
#ifdef USE_CUDA
        for (unsigned int camera_index=0;camera_index<raw.size();camera_index++) {
            gpu_zoom_streams[camera_index].waitForCompletion();
        }
#else
        if (zoom_thread.joinable()) zoom_thread.join();
#endif
        return zoom;
    }

    void Composite::set_zoom_size(const cv::Size &s) {
        zoom_size = s;

    }

    cv::Rect_<int> Composite::get_zoom_rect(cv::Size rs, cv::Size zs, cv::Point2f point, cv::Point2f &oftl) {
        cv::Point2f offset(zs.width / 2,zs.height / 2);
        auto tl = point - offset;
        auto br = point + offset;
        oftl = cv::Point2f (0,0);
        cv::Size ofbr (0,0);
        if (tl.x<0) {
            oftl.x = tl.x;
            ofbr.width = -tl.x;
        }
        if (tl.y<0) {
            oftl.y = tl.y;
            ofbr.height = -tl.y;
        }
        if (br.x > rs.width) {
            ofbr.width = br.x - rs.width;
        }
        if (br.y > rs.height) {
            ofbr.height = br.y - rs.height;
        }
        return cv::Rect_<int>(tl-oftl, zs-ofbr);
    }

    Image &Composite::get_video() {
#ifdef USE_CUDA
        gpu_video_stream.waitForCompletion();
#endif
        composite_video = composite.to_rgb();
        return composite_video;
    }

    void Composite::start_zoom(const Location &location) {
#ifdef USE_CUDA
        auto mouse_point = composite.get_point(location);
        for (unsigned int camera_index=0;camera_index<raw.size();camera_index++) {
            auto raw_point = get_raw_point(camera_index, mouse_point);
            cv::Point2f offset(0, 0);
            cv::Rect_<int> source = get_zoom_rect(size, zoom_size, raw_point, offset);
            cv::Rect_<int> destination(-offset, source.size());
            gpu_zoom[camera_index].setTo(0, gpu_zoom_streams[camera_index]);
            if (destination.height > 0 && destination.width > 0 && destination.x > 0 && destination.y > 0)
                gpu_raw[camera_index](source).copyTo(gpu_zoom[camera_index](destination), gpu_zoom_streams[camera_index]);
            gpu_raw[camera_index].download(zoom[camera_index], gpu_zoom_streams[camera_index]);
        }
#else
        zoom_thread = thread ([this](Location location) {
            auto mouse_point = composite.get_point(location);
            for (unsigned int camera_index = 0; camera_index < raw.size(); camera_index++) {
                auto raw_point = get_raw_point(camera_index, mouse_point);
                cv::Point2f offset(0, 0);
                cv::Rect_<int> source = get_zoom_rect(size, zoom_size, raw_point, offset);
                cv::Rect_<int> destination(-offset, source.size());
                zoom[camera_index].clear();
                if (destination.height > 0 && destination.width > 0 && destination.x > 0 && destination.y > 0)
                    raw[camera_index](source).copyTo(zoom[camera_index](destination));
            }
        }, location);
#endif
    }

}
