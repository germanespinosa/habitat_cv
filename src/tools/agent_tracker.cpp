#include <mutex>
#include <habitat_cv/cv_service.h>
#include <easy_tcp.h>
#include <experiment/experiment_client.h>
#include <sstream>
#include <iomanip>
#include <controller.h>
#include <robot_lib.h>
#include <experiment/experiment_service.h>
#include <params_cpp.h>

using namespace controller;
using namespace cell_world;
using namespace std;
using namespace json_cpp;
using namespace agent_tracking;
using namespace habitat_cv;
using namespace experiment;

struct Agent_tracker_configuration : Json_object {
    Json_object_members(
            Add_member(logs_folder);
            Add_member(videos_folder);
            Add_member(backgrounds_folder);
            Add_member(config_folder);
            )
    string logs_folder;
    string videos_folder;
    string backgrounds_folder;
    string config_folder;
};

int main(int argc, char **argv){

    params_cpp::Parser p(argc,argv);

    Agent_tracker_configuration config;
    config.load("../config/agent_tracker_config.json");

    controller::Agent_operational_limits limits;
    limits.load("../config/robot_operational_limits.json"); // robot, ghost

    auto occlusions_str = p.get(params_cpp::Key("-w", "--world"),"21_05");
    auto configuration = Resources::from("world_configuration").key("hexagonal").get_resource<World_configuration>();
    auto implementation = Resources::from("world_implementation").key("hexagonal").key("canonical").get_resource<World_implementation>(); // mice, vr, canonical
    auto occlusions = Resources::from("cell_group").key("hexagonal").key(occlusions_str).key("occlusions").get_resource<Cell_group_builder>();
    auto capture_parameters = Resources::from("capture_parameters").key("default").get_resource<Capture_parameters>();
    auto peeking_parameters = Resources::from("peeking_parameters").key("default").get_resource<Peeking_parameters>();

    auto world = World(configuration, implementation, occlusions);
    auto cells = world.create_cell_group();
    Map map(cells);
    Location_visibility visibility(cells, configuration.cell_shape, implementation.cell_transformation);
    Capture capture(capture_parameters, world);
    Peeking peeking(peeking_parameters, world);

    experiment::Experiment_server experiment_server;
    Experiment_service::set_logs_folder(config.logs_folder);
    experiment_server.start(Experiment_service::get_port()); // added by gabbie // 4540

    Tracking_server tracking_server;
    string cam_config = p.get(params_cpp::Key("-pc","--pixci_config"), "Default");
    string cam_file;
    cam_file = config.config_folder + "/EPIX_" + cam_config + ".fmt";
    string bg_path = config.backgrounds_folder + cam_config + "/";

    auto &experiment_client = experiment_server.create_local_client<Cv_server_experiment_client>();
    experiment_client.subscribe();

    auto homography_file = "homography_" + p.get(params_cpp::Key("-h","--homography"), "hab1");
    auto camera_configuration = json_cpp::Json_from_file<Camera_configuration>(config.config_folder + homography_file + ".json");

    auto sync_led_locations_file = "sync_led_locations_" + p.get(params_cpp::Key("-sll","--sync_led_locations"), "hab2");
    auto sync_led_locations = json_cpp::Json_from_file<Location_list>(config.config_folder + sync_led_locations_file + ".json");

    Cv_server cv_server(camera_configuration, cam_file, bg_path, config.videos_folder, tracking_server, experiment_client, sync_led_locations, capture_parameters, p.contains(params_cpp::Key("-u")));
    auto &experiment_tracking_client = tracking_server.create_local_client<Experiment_tracking_client>();
    experiment_tracking_client.subscribe();
    experiment_server.set_tracking_client(experiment_tracking_client);

    cv_server.occlusions = world.create_cell_group().occluded_cells();

    World_info wi;
    wi.world_configuration = "hexagonal";
    wi.world_implementation = "mice";
    wi.occlusions = "00_00";

    auto &controller_tracking_client = tracking_server.create_local_client<Controller_server::Controller_tracking_client>(
            visibility,
            float(360), //180 degrees each side -- sounds good?
            capture,
            peeking,
            "predator",
            "prey");
    controller_tracking_client.subscribe();


    auto &controller_experiment_client = experiment_server.create_local_client<Controller_server::Controller_experiment_client>();
    controller_experiment_client.subscribe();

//    robot::Robot_agent robot(limits);

//    if (!robot.connect("192.168.137.155")){
//        cout << "Failed to connect to predator robot" << endl;
//        //exit(1);
//    }

    Tick_agent_moves tick_moves;
    tick_moves.load("../config/tick_robot_moves.json");
    std::string joystick_path = "/dev/input/js0";

    auto &prey_tracking_client = tracking_server.create_local_client<Tracking_client>();
    prey_tracking_client.subscribe();
    robot::Tick_robot_agent prey_robot(tick_moves, prey_tracking_client, joystick_path);

    if (!p.contains(params_cpp::Key("-n"))) {
        if (!prey_robot.connect("192.168.137.155")) {
            cout << "Failed to connect to prey robot" << endl;
            //exit(1);
        }
    }
    Controller_service::set_logs_folder("controller/");
    Controller_server controller_server("../config/pid.json", prey_robot, controller_tracking_client, controller_experiment_client);

    if (!controller_server.start(Controller_service::get_port())) {
        cout << "failed to start controller" << endl;
        exit(1);
    }

//     initial corrector
    tracking_server.start(Tracking_service::get_port());
    auto t = std::thread([&prey_robot, &prey_tracking_client]() {
        while (!(prey_tracking_client.contains_agent_state("predator"))) this_thread::sleep_for(10ms);
        prey_robot.correct_robot();
    });

    tracking_server.start(Tracking_service::get_port());
    cv_server.tracking_process();
    tracking_server.stop();
    experiment_client.disconnect();
    exit(0);
}

// 2. check current tick values - can do this in a test file - or with old code
// 3. tune closed loop
// how to initialize
// get next move
// astar revamp
// autorobot rewrite
// 1. check message communication

// TODO: check to make sure angle readings from track are accurate