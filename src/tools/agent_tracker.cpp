#include <mutex>
#include <habitat_cv/cv_service.h>
#include <easy_tcp.h>
#include <experiment/experiment_client.h>
#include <sstream>
#include <iomanip>

using namespace cell_world;
using namespace std;
using namespace json_cpp;
using namespace agent_tracking;
using namespace habitat_cv;
using namespace experiment;

struct My_client : Experiment_client{
    void on_episode_started(const string &experiment_name) override{
        thread t([this](const string &experiment_name){
            auto experiment = this->get_experiment(experiment_name);
            std::stringstream ss;
            ss << "/habitat/videos/" << experiment_name << "/episode_" << std::setw(3) << std::setfill('0') << experiment.episode_count;
            string destination_folder = ss.str();
            Cv_service::new_episode(experiment.subject_name, experiment_name, experiment.episode_count,"", destination_folder);
        }, experiment_name);
        t.detach();
    }

    void on_episode_finished() override {
        Cv_service::end_episode();
    }
};

int main(int argc, char **argv){
    if (argc==1){
        cerr << "missing camera configuration parameter." << endl;
        exit(1);
    }
    My_client experiment_client;
    experiment_client.connect("127.0.0.1");
    experiment_client.subscribe();

    easy_tcp::Server<Cv_service> server;
    string cam_config = argv[1];
    string bg_path = "/habitat/habitat_cv/backgrounds/" + cam_config + "/";
    string cam_file = "/habitat/habitat_cv/config/EPIX_" + cam_config + ".fmt";
    World_info wi;
    wi.world_configuration = "hexagonal";
    wi.world_implementation = "cv";
    wi.occlusions = "00_00";

    Cv_service::set_world(wi);
    Cv_service::set_camera_file(cam_file);
    Cv_service::set_background_path(bg_path);
    server.start(Cv_service::get_port());
    Cv_service::tracking_process();
    experiment_client.disconnect();
    exit(0);
}