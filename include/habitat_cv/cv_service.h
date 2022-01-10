#pragma once
#include <agent_tracking/service.h>

namespace habitat_cv{
    struct Cv_service : agent_tracking::Service {
        //experiment
        bool new_experiment(const std::string &) override;
        bool new_episode(agent_tracking::New_episode_message) override;
        bool end_episode() override;
        bool update_puff() override;

        static void set_camera_file(const std::string &);
        static void set_background_path(const std::string &);
        static void set_occlusions(const std::string &);
        static void tracking_process();
        static void reset_cameras();
        static void initialize_background();
    };
}