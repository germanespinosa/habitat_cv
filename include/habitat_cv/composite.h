#pragma once
#include <habitat_cv/image.h>
#include <cell_world.h>
#include <habitat_cv/camera_configuration.h>
#include <thread>
#ifdef USE_CUDA
#include <thread>
#define cudaStreamNonBlocking 0x01
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudawarping.hpp>
#endif

namespace habitat_cv{
    struct Composite {
        Composite(Camera_configuration camera_configuration);
        Image &get_composite (const Images &);
        Image &get_video();
        cell_world::Polygon &get_polygon(const cell_world::Coordinates &);
        cell_world::Coordinates get_coordinates(const cell_world::Location &);
        cv::Point2f get_raw_point(unsigned int camera_index, const cv::Point2f &);
        cv::Point2f get_warped_point(unsigned int camera_index, const cv::Point2f &);
        cv::Rect_<int> get_zoom_rect(cv::Size rs, cv::Size zs, cv::Point2f point, cv::Point2f &);
        void set_zoom_size(const cv::Size &);
        Images &get_zoom();
        void start_zoom(const cell_world::Location &);

        Image composite_detection;
        Image composite;

#ifdef USE_CUDA
        cv::cuda::Stream gpu_video_stream;
        std::vector<cv::cuda::Stream> gpu_streams;
        std::vector<cv::cuda::GpuMat> gpu_raw;
        std::vector<cv::cuda::GpuMat> gpu_warped;
        std::vector<cv::cuda::GpuMat> gpu_warped_detection;
        std::vector<cv::cuda::GpuMat> gpu_warped_video;

        std::vector<cv::cuda::Stream> gpu_zoom_streams;
        std::vector<cv::cuda::GpuMat> gpu_zoom;

        cv::cuda::GpuMat gpu_composite;
        cv::cuda::GpuMat gpu_composite_detection;
        cv::cuda::GpuMat gpu_composite_video;
        cv::cuda::GpuMat gpu_composite_video_rgb;
        cv::cuda::GpuMat gpu_mask_detection;
        cv::cuda::GpuMat gpu_mask_video;
#endif
        std::thread zoom_thread;
        Images raw;
        Binary_image mask_detection;
        Binary_image mask_video;
        cv::Size size;
        Images zoom;
        std::vector<cv::Mat> homographies;
        std::vector<cv::Mat> inverted_homographies;
        Images warped_video;
        std::vector<cv::Rect> crop_rectangles;
        cell_world::World world;
        cell_world::Map map;
        cell_world::Polygon_list cells;
        Camera_configuration configuration;
        cv::Size zoom_size{150,150};
        Image &get_camera(unsigned int);
    private:
        Image composite_video;
        Images warped;
        Images warped_detection;
    };
}