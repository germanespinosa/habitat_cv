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

int main(int argc, char **argv){
    controller::Agent_operational_limits limits;
    limits.load("../config/robot_operational_limits.json"); // robot, ghost

    auto configuration = Resources::from("world_configuration").key("hexagonal").get_resource<World_configuration>();
    auto implementation = Resources::from("world_implementation").key("hexagonal").key("canonical").get_resource<World_implementation>(); // mice, vr, canonical
    auto capture_parameters = Resources::from("capture_parameters").key("default").get_resource<Capture_parameters>();
    auto peeking_parameters = Resources::from("peeking_parameters").key("default").get_resource<Peeking_parameters>();

    auto world = World(configuration, implementation);
    auto cells = world.create_cell_group();
    Map map(cells);
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

    robot::Robot_agent robot(limits);

//    if (!robot.connect("192.168.137.155")){
//        cout << "Failed to connect to predator robot" << endl;
//        //exit(1);
//    }

    Tick_agent_moves tick_moves;
    tick_moves.load("../config/tick_robot_moves.json");

    auto &prey_tracking_client = tracking_server.create_local_client<Tracking_client>();
    prey_tracking_client.subscribe();
    robot::Tick_robot_agent prey_robot(tick_moves,map, prey_tracking_client);

    if (!prey_robot.connect("192.168.137.154")){
        cout << "Failed to connect to prey robot" << endl;
        exit(1);
    }

    Controller_service::set_logs_folder("controller/");
    Controller_server controller_server("../config/pid.json", robot, controller_tracking_client, controller_experiment_client);

    if (!controller_server.start(Controller_service::get_port())) {
        cout << "failed to start controller" << endl;
        exit(1);
    }


    // PATH EXECUTION TEST
    bool prey_running = true;
    thread prey_wtf([&prey_robot, &prey_running]( ){
        //unsigned int i = 0;
        int input;
        int step;


        // path to execute
        vector<int> path0 = {0, 0,  3, 3, 3, 3, 0, 0, 0, 0,  3, 3, 3, 3, 0, 0,
                             0, 0,  3, 3, 3, 3, 0, 0, 0, 0,  3, 3, 3, 3, 0, 0}; // x correct analysis

        // LIST MOVES HERE
        Move_list moves;
        moves.emplace_back(2,0);
        moves.emplace_back(1,-1);
        moves.emplace_back(-1,-1);
        moves.emplace_back(-2,0);
        moves.emplace_back(-1,1);
        moves.emplace_back(1,1);


        // input -1 = exit
        // input 20 = sequence of moves


        prey_robot.move_count_reset();
        while (prey_running) {
            cout << "Enter move request: " << endl;
            cin >> input;
            if (input == 20){
                for (unsigned int j = 0; j < path0.size(); j++)
                {
                    step = path0[j];
                    auto move = moves[step];
                    prey_robot.execute_move(move);
                }
            } else {
                auto move = moves[input];
                prey_robot.execute_move(move);
            }

            if (input == -1) {
                prey_running = false;
                //cout << "Completed: " << prey_robot.completed_move << "Counter: " << prey_robot.move_counter << endl;
            }
        }
    });


    tracking_server.start(Tracking_service::get_port());
    cv_server.tracking_process();
    tracking_server.stop();
    experiment_client.disconnect();
    prey_running = false;
    if (prey_wtf.joinable()) prey_wtf.join();
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