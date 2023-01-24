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

struct Frame_location : json_cpp::Json_object {
    Json_object_members(
            Add_member(frame_number);
            Add_member(mouse);
            Add_member(robot);
            Add_member(puff);
            Add_member(mouse_location);
            )
    unsigned int frame_number;
    bool mouse;
    bool robot;
    bool puff;
    Location mouse_location;
};

struct Reverse_layout : habitat_cv::Layout {
    Reverse_layout() :
            Layout(1125, 1080, Image::Type::rgb),
            composite({0.0, 0.0}, {1080.0, 1124.0}, Image::Type::rgb),
            frame({280, 45}, Image::Type::rgb, {255, 255, 255}, {30, 30, 30}, .8, 2, 1) {
        add_place_holder(composite, {0, 0});
        add_place_holder(frame, {800, 990});
    }
    habitat_cv::Content_crop composite;
    habitat_cv::Content_text frame;
    habitat_cv::Image get_frame(const habitat_cv::Image &image, unsigned int frame_count) {
        composite = image;
        frame = "Frame: " + to_string(frame_count) + "  ";
        return get_image();
    }
};



int main(int argc, char **argv){
    params_cpp::Parser p(argc,argv);
    json_cpp::Json_vector<Frame_location> frame_locations;
    Image frame;
    auto vc = cv::VideoCapture(p.get(0));
    cv::Scalar puff_color {0, 0, 255};
    cv::Scalar robot_color {150, 0, 150};
    cv::Scalar mouse_color {120, 120, 0};
    cv::Mat base(Size{1080,985}, CV_8UC3, mouse_color);
    cv::Mat plain(Size{1080,985}, CV_8UC3, Scalar(255,255,255));
    cv::Rect r(Point{0,95+45}, Size{1080,984});
    Profile mouse;
    mouse.area_lower_bound = 10;
    mouse.area_upper_bound = 200;
    Profile robot;
    robot.area_lower_bound = 100;
    robot.area_upper_bound = 800;
    unsigned int frame_number = 0;
    Reverse_layout rl;
    Video output(rl.size(), rl.type);
    output.new_video(p.get(2));
    bool is_puffing = false;
    unsigned int robot_counter = 0;
    unsigned int mouse_counter = 0;
    unsigned int puff_counter = 0;
    while (vc.isOpened()) {
        vc.read(frame);
        if (!frame.empty()){
            Image sub;
            auto cropped = frame(r);
            Frame_location fl;
            fl.frame_number = frame_number;

            cv::absdiff(cropped, mouse_color, sub);
            Binary_image clean_mouse(sub.to_gray() < 25);
//            imshow("mouse", clean_mouse);
            auto detections = Detection_list::get_detections(clean_mouse);
            auto mouse_detections = detections.filter(mouse);
            if (!mouse_detections.empty()){
                mouse_counter++;
                fl.mouse = true;
                auto mouse_location = mouse_detections[0].location;
                mouse_location.x = mouse_location.x / 1080;
                mouse_location.y = (mouse_location.y - 985 / 2) / 1080 + .5;
                fl.mouse_location = mouse_location;
            } else {
                fl.mouse = false;
            }

            cv::absdiff(cropped, robot_color, sub);
            Binary_image clean_robot(sub.to_gray() < 25);
            detections = Detection_list::get_detections(clean_robot);
            auto robot_detections = detections.filter(robot);
            if (!robot_detections.empty()){
//                imshow("robot", clean_robot);
                robot_counter++;
                fl.robot = true;
            } else {
                cv::absdiff(cropped, puff_color, sub);
                Binary_image clean_puff(sub.to_gray() < 25);
                detections = Detection_list::get_detections(clean_puff);
                auto puff_detections = detections.filter(robot);
                if (!puff_detections.empty()){
//                    imshow("robot", clean_puff);
                    fl.robot = true;
                    fl.puff = !is_puffing;
                    if (fl.puff){
                        puff_counter++;
                        cout << "puff "<< endl;
                    }
                    is_puffing = true;
                } else {
//                    imshow("robot", clean_robot);
                    fl.robot = false;
                    fl.puff = false;
                    is_puffing = false;
                }
            }
//            waitKey(1);
            frame_locations.push_back(fl);
            auto new_frame = rl.get_frame(frame, frame_number);
            output.add_frame(new_frame);
//            cv::imshow("cropped_frame", new_frame);
//            cv::imshow("detection", clean);
//            cv::waitKey(1);
        } else {
            break;
        }
        frame_number ++;
    }
//    cout << "frames: " << frame_number << endl;
//    cout << "mouse: " << mouse_counter << endl;
//    cout << "robot: " << robot_counter << endl;
//    cout << "puff: " << puff_counter << endl;
    output.close();
    frame_locations.save(p.get(1));

    return 0;
}