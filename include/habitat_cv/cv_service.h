#pragma once
#include <agent_tracking/tracking_service.h>

namespace habitat_cv{
    struct Cv_service : agent_tracking::Tracking_service {
        static void set_camera_file(const std::string &);
        static void set_background_path(const std::string &);
        static void tracking_process(agent_tracking::Tracking_server &);
        static bool new_episode(const std::string &subject, const std::string &experiment, int episode, const std::string &occlusions, const std::string &destination_folder);
        static bool end_episode();
    };
}