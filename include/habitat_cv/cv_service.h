#pragma once
#include <habitat_cv/camera_configuration.h>
#include <habitat_cv/camera_array.h>
#include <habitat_cv/image.h>
#include <habitat_cv/layouts.h>
#include <habitat_cv/background.h>
#include <habitat_cv/composite.h>
#include <habitat_cv/video.h>
#include <habitat_cv/detection.h>
#include <habitat_cv/frame_rate.h>
#include <agent_tracking/tracking_service.h>


namespace habitat_cv{
    struct Cv_server {
        explicit Cv_server(const std::string &camera_configuration_file, const std::string &background_path, agent_tracking::Tracking_server &);
        void tracking_process();
        bool new_episode(const std::string &subject, const std::string &experiment, int episode, const std::string &occlusions, const std::string &destination_folder);
        bool get_mouse_step(const Image &diff, cell_world::Step &step, const cell_world::Location &robot_location);
        bool get_robot_step(const Image &image, cell_world::Step &step);
        bool end_episode();

        agent_tracking::Tracking_server &tracking_server;

        unsigned int robot_threshold = 240;

        cell_world::Space canonical_space;
        cell_world::Space cv_space;

        cell_world::Timer ts;
        std::atomic<bool> tracking_running = false;
        std::atomic<int> puff_state = false;
        cell_world::Cell_group occlusions;


        Camera_configuration camera_configuration;
        Camera_array cameras;
        Composite composite;


        Background background;
        Screen_layout screen_layout;
        Main_layout main_layout;
        Raw_layout raw_layout;
        Mouse_layout mouse_layout;


        Video main_video;
        Video raw_video;
        std::vector<Video *> mouse_videos;

        Profile led_profile;
        Profile mouse_profile;
    };
}