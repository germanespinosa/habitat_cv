#include <mutex>
#include <habitat_cv/cv_service.h>
#include <easy_tcp.h>

using namespace cell_world;
using namespace std;
using namespace json_cpp;
using namespace agent_tracking;
using namespace habitat_cv;

int main(int argc, char **argv){
    if (argc==1){
        cerr << "missing camera configuration parameter." << endl;
        exit(1);
    }
    easy_tcp::Server<Cv_service> server;
    string cam_config = argv[1];
    string bg_path = "/habitat/habitat_cv/backgrounds/" + cam_config + "/";
    string cam_file = "/habitat/habitat_cv/config/EPIX_" + cam_config + ".fmt";
    Cv_service::set_world_configuration("hexagonal");
    Cv_service::set_world_implementation("CV");
    Cv_service::set_camera_file(cam_file);
    Cv_service::set_background_path(bg_path);
    server.start(Service::get_port());
    Cv_service::tracking_process();
    exit(0);
}