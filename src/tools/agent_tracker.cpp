#include <mutex>
#include <habitat_cv/cv_service.h>
#include <easy_tcp.h>
#include <experiment/experiment_client.h>
#include <sstream>
#include <iomanip>
#include <controller.h>
#include <robot_lib.h>
#include <experiment/experiment_service.h>

using namespace controller;
using namespace cell_world;
using namespace std;
using namespace json_cpp;
using namespace agent_tracking;
using namespace habitat_cv;
using namespace experiment;

// something wrong with experiment name?
struct My_client : Experiment_client{
    void on_episode_started(const string &experiment_name) override{
        thread t([this](const string &experiment_name){
            auto experiment = this->get_experiment(experiment_name);
            std::stringstream ss;
            ss << "/habitat/videos/" << experiment_name << "/episode_" << std::setw(3) << std::setfill('0') << experiment.episode_count;
            string destination_folder = ss.str();
            Cv_service::new_episode(experiment.subject_name, experiment_name, experiment.episode_count, "", destination_folder);
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

    controller::Agent_operational_limits limits;
    limits.load("../config/robot_operational_limits.json"); // robot, ghost

    auto configuration = Resources::from("world_configuration").key("hexagonal").get_resource<World_configuration>();
    auto implementation = Resources::from("world_implementation").key("hexagonal").key("canonical").get_resource<World_implementation>(); // mice, vr, canonical
    auto capture_parameters = Resources::from("capture_parameters").key("default").get_resource<Capture_parameters>();
    auto peeking_parameters = Resources::from("peeking_parameters").key("default").get_resource<Peeking_parameters>();

    auto world = World(configuration, implementation);
    auto cells = world.create_cell_group();
    Location_visibility visibility(cells, configuration.cell_shape, implementation.cell_transformation);
    Capture capture(capture_parameters, world);
    Peeking peeking(peeking_parameters, world);

    experiment::Experiment_server experiment_server;
    Experiment_service::set_logs_folder("experiment/");
    experiment_server.start(Experiment_service::get_port()); // added by gabbie // 4540

    auto &experiment_client = experiment_server.create_local_client<My_client>();
    experiment_client.subscribe();


    Tracking_server server;
    string cam_config = argv[1];
    string bg_path = "/habitat/habitat_cv/backgrounds/" + cam_config + "/";
    string cam_file = "/habitat/habitat_cv/config/EPIX_" + cam_config + ".fmt";
    World_info wi;
    wi.world_configuration = "hexagonal";
    wi.world_implementation = "mice";
    wi.occlusions = "00_00";



    auto &controller_tracking_client = server.create_local_client<Controller_server::Controller_tracking_client>(
            visibility,
            float(90),
            capture,
            peeking,
            "predator",
            "prey");
    controller_tracking_client.subscribe();

    auto &controller_experiment_client =experiment_server.create_local_client<Controller_server::Controller_experiment_client>();
    controller_experiment_client.subscribe();


    robot::Robot_agent robot(limits);
    robot.connect("192.168.137.155");
    Controller_server controller("../config/pid.json", robot, controller_tracking_client, controller_experiment_client);
    Controller_service::set_logs_folder("controller/");
    controller.start(Controller_service::get_port());

    //Cv_service::set_world(wi);
    Cv_service::set_camera_file(cam_file);
    Cv_service::set_background_path(bg_path);
    server.start(Cv_service::get_port());
    Cv_service::tracking_process(server);
    experiment_client.disconnect();
    exit(0);
}