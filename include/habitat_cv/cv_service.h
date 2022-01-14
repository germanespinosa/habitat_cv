#pragma once
#include <agent_tracking/message.h>
#include <agent_tracking/tracking_service.h>

namespace habitat_cv{
    struct Cv_service : agent_tracking::Tracking_service {
        static void set_camera_file(const std::string &);
        static void set_background_path(const std::string &);
        static void tracking_process();
        bool new_episode(agent_tracking::New_episode_message nem);
        bool end_episode();
    };
}