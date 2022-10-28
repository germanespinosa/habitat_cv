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

    struct Predator_data : json_cpp::Json_object{
        Json_object_members(
                Add_member(capture);
                Add_member(human_intervention);
                Add_member(best_camera);
                )
        bool capture;
        bool human_intervention;
        int best_camera;
    };

    struct Cv_server_experiment_client : experiment::Experiment_client{
        explicit Cv_server_experiment_client();
        void on_episode_started(const std::string &experiment_name) override;
        void on_capture(int frame) override;
        void on_episode_finished() override;
        void on_experiment_started(const experiment::Start_experiment_response &) override;
        void on_human_intervention(bool) override;
        Cv_server *cv_server;
    };

    struct Cv_server {
        explicit Cv_server(const Camera_configuration &camera_configuration,
                           const std::string &camera_configuration_file,
                           const std::string &background_path,
                           const std::string &video_path,
                           agent_tracking::Tracking_server &,
                           Cv_server_experiment_client &,
                           bool unlimited = false);
        void tracking_process();
        bool new_episode(const std::string &subject, const std::string &experiment, int episode, const std::string &occlusions, const std::string &destination_folder);
        bool get_mouse_step(const Binary_image &diff, cell_world::Step &step, const cell_world::Location &robot_location, float scale);
        bool get_robot_step(const Binary_image &image, cell_world::Step &step, float scale);
        bool end_episode();

        agent_tracking::Tracking_server &tracking_server;
        Cv_server_experiment_client &experiment_client;

        unsigned int mouse_threshold = 55;
        unsigned int robot_threshold = 250;

        cell_world::Space canonical_space;
        cell_world::Space cv_space;
        bool human_intervention = false;
        cell_world::Timer ts;
        std::atomic<bool> tracking_running = false;
        std::atomic<int> puff_state = 0;
        cell_world::Cell_group occlusions;

        bool unlimited;
        bool waiting_for_prey = false;
        Camera_configuration camera_configuration;
        Camera_array cameras;


        Background background;
        Screen_layout screen_layout;
        Main_layout main_layout;
        Raw_layout raw_layout;
        Mouse_layout mouse_layout;


        Video main_video;
        Video raw_video;
        Video zoom_video;

        Profile led_profile;
        Profile mouse_profile;
        std::string video_path;
        std::string background_path;

        float robot_height = 10.4;    // cm  // 5 (short) , 10.4 (tall)
        std::vector<cv::Point2f> zoom_rectangles;
        cv::Size zoom_size{150,150};

    };
}