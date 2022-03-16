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


struct My_client : Experiment_client{
    explicit My_client(Cv_server *cv_server):
    cv_server(cv_server){};
    Cv_server *cv_server;
    void on_episode_started(const string &experiment_name) override{
        auto experiment = this->get_experiment(experiment_name);
        std::stringstream ss;
        ss << "/habitat/videos/" << experiment_name << "/episode_" << std::setw(3) << std::setfill('0') << experiment.episode_count;
        string destination_folder = ss.str();
        cv_server->new_episode(experiment.subject_name, experiment_name, experiment.episode_count, "", destination_folder);
    }

    void on_episode_finished() override {
        cv_server->end_episode();
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

    Tracking_server tracking_server;
    string cam_config = argv[1];
    string bg_path = "/habitat/habitat_cv/backgrounds/" + cam_config + "/";
    string cam_file = "/habitat/habitat_cv/config/EPIX_" + cam_config + ".fmt";
    Cv_server cv_server(cam_file, bg_path, tracking_server);

    auto &experiment_client = experiment_server.create_local_client<My_client>(&cv_server);
    experiment_client.subscribe();


    experiment_server.set_tracking_client(tracking_server.create_local_client<Experiment_tracking_client>());

    World_info wi;
    wi.world_configuration = "hexagonal";
    wi.world_implementation = "mice";
    wi.occlusions = "00_00";

    auto &controller_tracking_client = tracking_server.create_local_client<Controller_server::Controller_tracking_client>(
            visibility,
            float(90),
            capture,
            peeking,
            "predator",
            "prey");
    controller_tracking_client.subscribe();

    auto &controller_experiment_client = experiment_server.create_local_client<Controller_server::Controller_experiment_client>();
    controller_experiment_client.subscribe();

    robot::Robot_agent robot(limits);


    if (!robot.connect("192.168.137.155")){
        cout << "Failed to connect to robot" << endl;
        exit(1);
    }

    Controller_service::set_logs_folder("controller/");
    Controller_server controller_server("../config/pid.json", robot, controller_tracking_client, controller_experiment_client);

    if (!controller_server.start(Controller_service::get_port())) {
        cout << "failed to start controller" << endl;
        exit(1);
    }
    tracking_server.start(Tracking_service::get_port());

    cv_server.tracking_process();
    tracking_server.stop();
    experiment_client.disconnect();
    exit(0);
}