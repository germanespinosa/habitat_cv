#include <thread>
#include <filesystem>
#include <habitat_cv/cv_service.h>


#define SAFETY_MARGIN 75
#define PUFF_DURATION 15

using namespace agent_tracking;
using namespace cell_world;
using namespace easy_tcp;
using namespace std;
using namespace tcp_messages;


#define ENTRANCE Location(0,.5)
#define  ENTRANCE_DISTANCE .05

namespace habitat_cv {

    bool Cv_server::new_episode(const string &subject, const string &experiment, int episode, const string &occlusions, const string &destination_folder) {
        cout << "new_episode" << endl;
        if (main_video.is_open()) end_episode();
        std::filesystem::create_directories(destination_folder);
        cout << "Video destination folder: " + destination_folder << endl;
        main_layout.new_episode(subject, experiment, episode, occlusions);
        main_video.new_video(destination_folder + "/main_" + experiment + ".mp4");
        raw_video.new_video(destination_folder + "/raw_" + experiment + ".mp4");
        for (int i=0; i<4; i++) {
            mouse_videos[i]->new_video(destination_folder + "/mouse" + to_string(i) + "_" + experiment + ".mp4");
        }
        ts.reset();
        waiting_for_prey = true;
        return true;
    }

    bool Cv_server::end_episode() {
        cout << "end_episode" << endl;
        main_video.close();
        for (auto &mouse_video:mouse_videos) {
            mouse_video->close();
        }
        raw_video.close();
        return true;
    }

    bool Cv_server::get_mouse_step(const Image &diff, Step &step, const Location &robot_location) {
        auto mouse_candidates = Detection_list::get_detections(diff, 55, 2).filter(mouse_profile);
        for (auto &mouse: mouse_candidates) {
            if (mouse.location.dist(robot_location) < SAFETY_MARGIN) continue;
            step.agent_name = "prey";
            step.location = mouse.location;
            return true;
        }
        return false;
    }

    bool Cv_server::get_robot_step(const Image &image, Step &step) {
        auto leds = Detection_list::get_detections(image, robot_threshold, 0).filter(led_profile);
        if (leds.size() != 3) return false;
        double d1 = leds[0].location.dist(leds[1].location);
        double d2 = leds[1].location.dist(leds[2].location);
        double d3 = leds[2].location.dist(leds[0].location);
        Location back;
        Location front;
        if (d1 < d2 && d1 < d3) {
            back = (leds[0].location + leds[1].location);
            back.x /= 2;
            back.y /= 2;
            front = leds[2].location;
        } else if (d2 < d3 && d2 < d1) {
            back = (leds[1].location + leds[2].location);
            back.x /= 2;
            back.y /= 2;
            front = leds[0].location;
        } else {
            back = (leds[2].location + leds[0].location);
            back.x /= 2;
            back.y /= 2;
            front = leds[1].location;
        }
        step.location.x = (front.x + back.x) / 2;
        step.location.y = (front.y + back.y) / 2;
        step.rotation = -to_degrees(atan2(front.y - back.y, front.x - back.x) - M_PI / 2);
        step.agent_name = "predator";
        return true;
    }


#define NOLOCATION Location(-1000,-1000)
    enum Screen_image {
        main,
        difference,
        led,
        led0,
        led1,
        led2,
        led3,
        raw,
        mouse,
        cam0,
        cam1,
        cam2,
        cam3,
        warped0,
        warped1,
        warped2,
        warped3,
        warped_video0,
        warped_video1,
        warped_video2,
        warped_video3,
        warped_detection0,
        warped_detection1,
        warped_detection2,
        warped_detection3,
        composite_image,
        large
    };

    void Cv_server::tracking_process() {
        tracking_running = true;
        puff_state = false;
        Step mouse;
        mouse.location = NOLOCATION;
        Step robot;
        robot.location = NOLOCATION;
        int robot_counter = 0;
        ts.reset();
        int robot_best_cam = -1;
        bool new_robot_data;
        string screen_text;
        Screen_image screen_image = Screen_image::main;
        double fps = Video::get_fps();
        double time_out = 1.0 / fps * .999;
        cout << "time_out: " << time_out << endl;
        float camera_height = 205;
        float robot_height = 12;
        float height_ratio = robot_height / camera_height;
        vector<Location> camera_zero;
        {
            int i = 0;
            auto zero_point = cv::Point2f(Camera::frame_size.width / 2, Camera::frame_size.height / 2);
            auto images = cameras.capture();
            //images.emplace_back(images[2],"camera_3.png");
            auto composite_image = composite.get_composite(images);
            for (auto &camera: cameras.cameras) {
                auto camera_zero_point = composite.get_warped_point(i++, zero_point);
                auto camera_zero_location = composite_image.get_location(camera_zero_point);
                camera_zero.push_back(camera_zero_location);
            }
            camera_zero.push_back(camera_zero[2]);
        }
        Timer frame_timer(time_out);
        Frame_rate fr;
        fr.filter = .99999;
        bool show_occlusions = false;
        while (tracking_running) {
            //Timer capture_timer;
            while (!frame_timer.time_out());
            frame_timer.reset();
            auto images = cameras.capture();
            //cout << capture_timer.to_seconds() * 1000 << " " ;
            //capture_timer.reset();
            //added to copy 3th camera into a 4th buffer (broken camera fix)
            //images.emplace_back(images[2],"camera_3.png");
            composite.get_composite(images);
            auto composite_image_gray = composite.composite_detection;
            auto composite_image_rgb = composite.composite_video.to_rgb();
            //cout << capture_timer.to_seconds() * 1000 << endl ;
            if (robot_best_cam == -1) {
                new_robot_data = get_robot_step(composite_image_gray, robot);
            } else {
                new_robot_data = get_robot_step(composite.warped_detection[robot_best_cam], robot);
                if (!new_robot_data) {
                    new_robot_data = get_robot_step(composite_image_gray, robot);
                }
            }
            unsigned int frame_number = 0;
            if (main_video.is_open()) {
                frame_number = main_video.frame_count;
            }
            if (new_robot_data) {
                if ((robot.location.x < 500 || robot.location.x > 580) &&
                    (robot.location.y < 500 || robot.location.y > 580)) {
                    int cam_row = robot.location.y > 540 ? 0 : 1;
                    int cam_col = robot.location.x > 540 ? 1 : 0;
                    robot_best_cam = camera_configuration.order[cam_row][cam_col];
                }
                auto perspective_offset = robot.location - camera_zero[robot_best_cam];
                auto perspective_adjustment = perspective_offset * height_ratio;
                robot.location += (-perspective_adjustment);
                auto color_robot = cv::Scalar({255, 0, 255});
                if (puff_state) {
                    robot.data = "puff";
                    color_robot = cv::Scalar({0, 0, 255});
                    puff_state--;
                } else {
                    robot.data = "";
                }
                thread([this, frame_number](Step &robot, Timer &ts, Tracking_server& tracking_server){
                    robot.time_stamp = ts.to_seconds();
                    robot.frame = frame_number;
                    tracking_server.send_step(robot.convert(cv_space,canonical_space));
                }, reference_wrapper(robot), reference_wrapper(ts), reference_wrapper(tracking_server)).detach();

                composite_image_rgb.circle(robot.location, 5, color_robot, true);
                auto robot_cell_id = composite.map.cells.find(robot.location);
                auto robot_cell_coordinates = composite.map.cells[robot_cell_id].coordinates;

                auto cell_polygon = composite.get_polygon(robot_cell_coordinates);
                composite_image_rgb.polygon(cell_polygon, color_robot);
                composite_image_rgb.arrow(robot.location, to_radians(robot.rotation), 50, color_robot);

                robot_counter = 30;
            } else {
                if (robot_counter) robot_counter--;
                else robot.location = NOLOCATION;
            }
            //for (auto &i : images)
                //cout << i.time_stamp.to_seconds() * 1000 << ", ";
            //cout << endl;
            auto diff = composite_image_gray.diff(background.composite);
            if (get_mouse_step(diff, mouse, robot.location)) {
                auto canonical_step = mouse.convert(cv_space,canonical_space);
                if (waiting_for_prey && canonical_step.location.dist(ENTRANCE) > ENTRANCE_DISTANCE) {
                    waiting_for_prey = false;
                    experiment_client.prey_enter_arena();
                }
                thread([this, frame_number](Step &canonical_step, Timer &ts, Tracking_server& tracking_server){
                    canonical_step.time_stamp = ts.to_seconds();
                    canonical_step.frame = frame_number;
                    tracking_server.send_step(canonical_step);
                }, reference_wrapper(canonical_step), reference_wrapper(ts), reference_wrapper(tracking_server)).detach();

                composite_image_rgb.circle(mouse.location, 5, {255, 0, 0}, true);
                auto mouse_cell_id = composite.map.cells.find(mouse.location);
                auto mouse_cell_coordinates = composite.map.cells[mouse_cell_id].coordinates;
                auto cell_polygon = composite.get_polygon(mouse_cell_coordinates);
                composite_image_rgb.polygon(cell_polygon, {255, 0, 0});
            }
            auto main_frame = main_layout.get_frame(composite_image_rgb, frame_number);

            auto raw_frame = raw_layout.get_frame(images);
            int camera_index = 0;
            Images mouse_cut;
            for (auto &image: images) {
                auto mouse_point = composite_image_gray.get_point(mouse.location);
                auto mouse_raw_point = composite.get_raw_point(camera_index,mouse_point);
                auto mouse_raw_location = image.get_location(mouse_raw_point);
                mouse_cut.emplace_back(Content_crop(mouse_raw_location,150, image));
                camera_index++;
            }
            auto mouse_frame = raw_layout.get_frame(mouse_cut);
            Image screen_frame;
            switch (screen_image) {
                case Screen_image::main :
                    if (show_occlusions) {
                        for (auto &occlusion: occlusions) {
                            composite_image_rgb.circle(occlusion.get().location, 20, {255, 0, 0}, true);
                        }
                    }
                    screen_frame = screen_layout.get_frame(composite_image_rgb, "main");
                    break;
                case Screen_image::difference :
                    screen_frame = screen_layout.get_frame(diff, "difference");
                    break;
                case Screen_image::led :
                    if (robot_best_cam == -1)
                        screen_frame = screen_layout.get_frame(Image(cv::Mat(composite_image_gray > robot_threshold),""), "LEDs");
                    else
                        screen_frame = screen_layout.get_frame(Image(cv::Mat(composite.warped_detection[robot_best_cam] > robot_threshold),""), "LEDs");
                    break;
                case Screen_image::led0 :
                    screen_frame = screen_layout.get_frame(Image(cv::Mat(composite.warped_detection[0] > robot_threshold),""), "LED0");
                    break;
                case Screen_image::led1 :
                    screen_frame = screen_layout.get_frame(Image(cv::Mat(composite.warped_detection[1] > robot_threshold),""), "LED1");
                    break;
                case Screen_image::led2 :
                    screen_frame = screen_layout.get_frame(Image(cv::Mat(composite.warped_detection[2] > robot_threshold),""), "LED2");
                    break;
                case Screen_image::led3 :
                    screen_frame = screen_layout.get_frame(Image(cv::Mat(composite.warped_detection[3] > robot_threshold),""), "LED3");
                    break;
                case Screen_image::mouse :
                    screen_frame = screen_layout.get_frame(mouse_frame, "mouse");
                    break;
                case Screen_image::raw :
                    screen_frame = screen_layout.get_frame(raw_frame, "raw");
                    break;
                case Screen_image::cam0 :
                    screen_frame = screen_layout.get_frame(images[0], "cam0");
                    break;
                case Screen_image::cam1 :
                    screen_frame = screen_layout.get_frame(images[1], "cam1");
                    break;
                case Screen_image::cam2 :
                    screen_frame = screen_layout.get_frame(images[2], "cam2");
                    break;
                case Screen_image::cam3 :
                    screen_frame = screen_layout.get_frame(images[3], "cam3");
                    break;
                case Screen_image::warped0 :
                    screen_frame = screen_layout.get_frame(composite.warped[0], "warped0");
                    break;
                case Screen_image::warped1 :
                    screen_frame = screen_layout.get_frame(composite.warped[1], "warped1");
                    break;
                case Screen_image::warped2 :
                    screen_frame = screen_layout.get_frame(composite.warped[2], "warped2");
                    break;
                case Screen_image::warped3 :
                    screen_frame = screen_layout.get_frame(composite.warped[3], "warped3");
                    break;
                case Screen_image::warped_video0 :
                    screen_frame = screen_layout.get_frame(composite.warped_video[0], "warped_video0");
                    break;
                case Screen_image::warped_video1 :
                    screen_frame = screen_layout.get_frame(composite.warped_video[1], "warped_video1");
                    break;
                case Screen_image::warped_video2 :
                    screen_frame = screen_layout.get_frame(composite.warped_video[2], "warped_video2");
                    break;
                case Screen_image::warped_video3 :
                    screen_frame = screen_layout.get_frame(composite.warped_video[3], "warped_video3");
                    break;
                case Screen_image::warped_detection0 :
                    screen_frame = screen_layout.get_frame(composite.warped_detection[0], "warped_detection0");
                    break;
                case Screen_image::warped_detection1 :
                    screen_frame = screen_layout.get_frame(composite.warped_detection[1], "warped_detection1");
                    break;
                case Screen_image::warped_detection2 :
                    screen_frame = screen_layout.get_frame(composite.warped_detection[2], "warped_detection2");
                    break;
                case Screen_image::warped_detection3 :
                    screen_frame = screen_layout.get_frame(composite.warped_detection[3], "warped_detection3");
                    break;
                case Screen_image::composite_image :
                    screen_frame = screen_layout.get_frame(composite.composite_video, "composite");
                    break;
                case Screen_image::large :
                    screen_frame = screen_layout.get_frame(composite.composite_video, "large");
                    break;
            }
            if (main_video.is_open()) screen_frame.circle({20, 20}, 10, {0, 0, 255}, true);
            cv::imshow("Agent Tracking", screen_frame);
            auto key = cv::waitKey(1);
            switch (key) {
                case 'C':
                    // start video recording
                    images.save(".");
                    break;
                case 'V':
                    // start video recording
                    main_video.new_video("main.mp4");
                    raw_video.new_video("raw.mp4");
                    for (int i=0; i<4; i++) {
                        mouse_videos[i]->new_video("mouse" + to_string(i) + ".mp4");
                    }
                    break;
                case 'B':
                    // end video recording
                    main_video.close();
                    for (auto &mouse_video:mouse_videos) {
                        mouse_video->close();
                    }
                    raw_video.close();
                    break;
                case 'Q':
                    tracking_running = false;
                    break;
                case 'M':
                    screen_image = main;
                    break;
                case 'R':
                    cameras.reset();
                    break;
                case '[':
                    robot_threshold++;
                    cout << "robot threshold set to " << robot_threshold << endl;
                    break;
                case ']':
                    robot_threshold--;
                    cout << "robot threshold set to " << robot_threshold << endl;
                    break;
                case 'U':
                    background.update(composite.composite_detection, composite.warped_detection);
                    break;
                case 'O':
                    show_occlusions = !show_occlusions;
                    break;
                case '\t':
                    if (screen_image == Screen_image::large)
                        screen_image = Screen_image::main;
                    else
                        screen_image = static_cast<Screen_image>(screen_image + 1);
                    cout << "change_screen_output to " << screen_image << endl;
                    break;
                case ' ':
                    if (screen_image == Screen_image::main)
                        screen_image = Screen_image::large;
                    else
                        screen_image = static_cast<Screen_image>(screen_image - 1);
                    cout << "change_screen_output to " << screen_image << endl;
                    break;
            }
            fr.new_frame();
            cout << fr.filtered_fps<< "  fps                 \r";
            if (mouse.location == NOLOCATION) continue; // starts recording when mouse crosses the door
            //thread t([this, main_frame, mouse_cut, raw_frame]() {
                main_video.add_frame(main_frame);
                raw_video.add_frame(raw_frame);
                for (int i=0;i<4;i++) {
                    mouse_videos[i]->add_frame(mouse_cut[i]);
                }
            //});
            //t.detach();
            if (!main_video.is_open()) mouse.location = NOLOCATION;
            // write videos
        }

    }

    Cv_server::Cv_server(const std::string &camera_configuration_file,
                         const std::string &background_path,
                         agent_tracking::Tracking_server &tracking_server,
                         Cv_server_experiment_client &experiment_client):
        tracking_server(tracking_server),
        experiment_client(experiment_client),
        canonical_space(World_implementation::get_from_parameters_name("hexagonal","canonical").space),
        cv_space(World_implementation::get_from_parameters_name("hexagonal","cv").space),
        camera_configuration(Resources::from("camera_configuration").key("default").get_resource<Camera_configuration>()),
        //cameras(camera_configuration_file, camera_configuration.order.count()),
        cameras(camera_configuration_file, 4),
        composite(camera_configuration),
        main_video(main_layout.size(), Image::rgb),
        raw_video(raw_layout.size(), Image::gray),
        led_profile(Resources::from("profile").key("led").get_resource<Profile>()),
        mouse_profile(Resources::from("profile").key("mouse").get_resource<Profile>())
{
        experiment_client.cv_server = this;
        for (int i = 0; i < 4; i++) {
            mouse_videos.push_back(new Video(cv::Size(150,150), Image::gray));
        }
        background.set_path(background_path);
        if (!background.load()) {
            auto images = cameras.capture();
            images.push_back(images[2]);
            composite.get_composite(images);
            background.update(composite.composite_detection, composite.warped_detection);
        }

}

    void Cv_server_experiment_client::on_episode_started(const string &experiment_name) {
        auto experiment = this->get_experiment(experiment_name);
        std::stringstream ss;
        ss << "/habitat/videos/" << experiment_name << "/episode_" << std::setw(3) << std::setfill('0') << experiment.episode_count;
        std::string destination_folder = ss.str();
        cv_server->new_episode(experiment.subject_name, experiment.experiment_name, experiment.episode_count, experiment.world_info.occlusions, destination_folder);
    }

    void Cv_server_experiment_client::on_episode_finished() {
        cv_server->end_episode();
    }

    Cv_server_experiment_client::Cv_server_experiment_client() {}

    void Cv_server_experiment_client::on_capture(int frame) {
        cv_server->puff_state =  PUFF_DURATION;
    }

    void Cv_server_experiment_client::on_experiment_started(const experiment::Start_experiment_response &experiment) {
        cv_server->occlusions = World::get_from_parameters_name(experiment.world.world_configuration,"cv",experiment.world.occlusions).create_cell_group().occluded_cells();
    }

}