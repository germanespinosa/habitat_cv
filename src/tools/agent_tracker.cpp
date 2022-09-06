// Note:
// 1. specify what robot to connect to sim or real
#include <mutex>
#include <habitat_cv/cv_service.h>
#include <easy_tcp.h>
#include <experiment/experiment_client.h>
#include <sstream>
#include <iomanip>
#include <controller.h>
#include <robot_lib.h>
#include <experiment/experiment_service.h>
#include <robot_lib/robot_agent.h>
#include <params_cpp.h>

using namespace controller;
using namespace cell_world;
using namespace std;
using namespace json_cpp;
using namespace agent_tracking;
using namespace habitat_cv;
using namespace experiment;
using namespace robot;
using namespace params_cpp;

int main(int argc, char **argv){

    Key robot_task_key{"-t", "--task"};
    Parser p(argc, argv);
    bool task = p.contains(robot_task_key);


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
    Experiment_service::set_logs_folder("/habitat/logsV2/");
    experiment_server.start(Experiment_service::get_port()); // added by gabbie // 4540

    Tracking_server tracking_server;
    string cam_config = argv[1];
    string bg_path = "/habitat/habitat_cv/backgrounds/" + cam_config + "/";
    string cam_file;
    if (argc==1){
        cam_file = "/usr/local/xcap/settings/xcvidset.fmt";
    } else {
        cam_file = "/habitat/habitat_cv/config/EPIX_" + cam_config + ".fmt";
    }

    auto &experiment_client = experiment_server.create_local_client<Cv_server_experiment_client>();
    experiment_client.subscribe();

    Cv_server cv_server(cam_file, bg_path, tracking_server, experiment_client);
    auto &experiment_tracking_client = tracking_server.create_local_client<Experiment_tracking_client>();
    experiment_tracking_client.subscribe();
    experiment_server.set_tracking_client(experiment_tracking_client);

    World_info wi;
    wi.world_configuration = "hexagonal";
    wi.world_implementation = "mice";
    wi.occlusions = "00_00";

    struct : Robot_agent {
        // this is move finished function see robot_agent to find move_number passed
        void move_finished(int move_number) override {
            if (controller_server)
                controller_server->broadcast_subscribed(tcp_messages::Message("move_finished", move_number));
        };
        Controller_server *controller_server = nullptr;
    } robot_agent;

    // 192.168.137.155
    if (!robot_agent.connect("192.168.137.155")){
        cout << "Failed to connect to robot" << endl;
        exit(1);
    }

    Controller_server *controller_server;

    if (task) {
        cout << "Start Controller Server" << endl;
        auto &controller_tracking_client = tracking_server.create_local_client<Controller_server::Controller_tracking_client>(
                visibility,
                float(90), //180 degrees each side -- sounds good?
                capture,
                peeking,
                "predator",
                "prey");
        controller_tracking_client.subscribe();

        auto &controller_experiment_client = experiment_server.create_local_client<Controller_server::Controller_experiment_client>();
        controller_experiment_client.subscribe();

        Controller_service::set_logs_folder("controller/");
        controller_server = new Controller_server("../config/pid.json", robot_agent, controller_tracking_client,
                                            controller_experiment_client);

        robot_agent.controller_server = controller_server;
        if (!controller_server->start(Controller_service::get_port())) {
            cout << "failed to start controller" << endl;
            exit(1);
        }
    }



    tracking_server.start(Tracking_service::get_port());

    cv_server.tracking_process();
    tracking_server.stop();
    experiment_client.disconnect();
    robot_agent.disconnect();
    exit(0);
}