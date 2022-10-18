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
#include <experiment/experiment_client.h>


namespace habitat_cv{
    struct Cv_server;

    struct Cv_server_experiment_client : experiment::Experiment_client{
        explicit Cv_server_experiment_client();
        void on_episode_started(const std::string &experiment_name) override;
        void on_capture(int frame) override;
        void on_episode_finished() override;
        void on_experiment_started(const experiment::Start_experiment_response &) override;
        Cv_server *cv_server;
    };

    struct Cv_server {
        explicit Cv_server(const std::string &camera_configuration_file, const std::string &background_path, agent_tracking::Tracking_server &, Cv_server_experiment_client &);
        void tracking_process();
        bool new_episode(const std::string &subject, const std::string &experiment, int episode, const std::string &occlusions, const std::string &destination_folder);
        bool get_mouse_step(const Image &diff, cell_world::Step &step, const cell_world::Location &robot_location);
        bool get_robot_step(const Image &image, cell_world::Step &step);
        bool end_episode();
        float get_prey_robot_orientation(Image &);

        agent_tracking::Tracking_server &tracking_server;
        Cv_server_experiment_client &experiment_client;

        unsigned int robot_threshold = 240;

        cell_world::Space canonical_space;
        cell_world::Space cv_space;

        cell_world::Timer ts;
        std::atomic<bool> tracking_running = false;
        std::atomic<int> puff_state = 0;
        cell_world::Cell_group occlusions;

        bool waiting_for_prey;
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
        Profile prey_robot_head_profile;
    };
}