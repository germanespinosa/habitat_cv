#include <opencv2/opencv.hpp>
#include <cell_world.h>
#include <habitat_cv/image.h>
#include <habitat_cv/detection.h>
#include <habitat_cv/layout.h>
#include <opencv2/video.hpp>
#include <params_cpp.h>
#include <habitat_cv/video.h>

using namespace cell_world;
using namespace cv;
using namespace habitat_cv;
using namespace std;


struct Control_layout : habitat_cv::Layout {
    Control_layout() :
            Layout(1125, 1080, Image::Type::rgb),
            composite({0.0, 0.0}, {1080.0, 1124.0}, Image::Type::rgb) {
        add_place_holder(composite, {0, 0});
    }
    habitat_cv::Content_crop composite;
    habitat_cv::Image get_frame(const habitat_cv::Image &image) {
        composite = image;
        return get_image();
    }
};

int main(int argc, char **argv){
    params_cpp::Parser p(argc,argv);
    Episode episode;
    episode.load(p.get(1));
    Trajectories mouse_trajectories = episode.trajectories.filter([](const Step &s){ return s.agent_name=="prey";});
    Trajectories robot_trajectories = episode.trajectories.filter([](const Step &s){ return s.agent_name=="predator";});

    auto vc = cv::VideoCapture(p.get(0) + ".mp4");
    Image frame;
    Control_layout cl;
    Video output (cl.size(), cl.type);
    output.new_video(p.get(0) + "_control");
    unsigned int frame_number = 0;
    while (vc.isOpened()) {
        vc.read(frame);
        if (!frame.empty()){
            try {
                auto mi = mouse_trajectories.find_first_index(
                        [frame_number](const Step &s) { return s.frame == frame_number; });
                if (mi >= 0) {
                    auto mouse_location = mouse_trajectories[mi].location;
                    mouse_location.x = mouse_location.x * 1080;
                    mouse_location.y = mouse_location.y * 1080 - 48;
                    frame.circle(mouse_location, 3, Scalar(0, 0, 255), true);
                }
            } catch(...){}
            try {
                auto ri = robot_trajectories.find_first_index(
                        [frame_number](const Step &s) { return s.frame == frame_number; });
                if (ri >= 0) {
                    auto robot_location = robot_trajectories[ri].location;
                    robot_location.x = robot_location.x * 1080;
                    robot_location.y = robot_location.y * 1080 - 48;
                    frame.circle(robot_location, 3, Scalar(0, 0, 255), true);
                }
            } catch (...) {}
            output.add_frame(cl.get_frame(frame));
//            auto a = frame.size();
//            cout << a.width << " " << a.height <<  endl;
        } else {
            break;
        }
        frame_number ++;
    }
    output.close();
    return 0;
}