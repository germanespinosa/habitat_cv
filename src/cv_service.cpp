#include <thread>
#include <filesystem>
#include <habitat_cv/cv_service.h>
#include <performance.h>


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

    bool Cv_server::new_episode(const string &subject,
                                const string &experiment,
                                int episode,
                                const string &occlusions,
                                const string &folder) {
        cout << "new_episode" << endl;
        auto destination_folder = video_path + folder;
        if (main_video.is_open()) end_episode();
        std::filesystem::create_directories(destination_folder);
        cout << "Video destination folder: " + destination_folder << endl;
        main_layout.new_episode(subject, experiment, episode, occlusions);
        main_video.new_video(destination_folder + "/main_" + experiment );
        raw_video.new_video(destination_folder + "/raw_" + experiment );
        zoom_video.new_video(destination_folder + "/mouse_" + experiment );
        ts.reset();
        waiting_for_prey = true;
        return true;
    }

    bool Cv_server::end_episode() {
        cout << "end_episode" << endl;
        main_video.close();
        zoom_video.close();
        raw_video.close();
        return true;
    }

    bool Cv_server::get_mouse_step(const Image &diff, Step &step, const Location &robot_location, float scale) {
        PERF_START("MOUSE_DETECTIONS");
        auto detections = Detection_list::get_detections(diff, 55, 1).scale(scale);
        PERF_STOP("MOUSE_DETECTIONS");
        PERF_START("MOUSE_FILTER");
        auto mouse_candidates = detections.filter(mouse_profile);
        PERF_STOP("MOUSE_FILTER");
        PERF_SCOPE("MOUSE_REST");
        for (auto &mouse: mouse_candidates) {
            if (mouse.location.dist(robot_location) < SAFETY_MARGIN) continue;
            step.agent_name = "prey";
            step.location = mouse.location;
            return true;
        }
        return false;
    }

    bool Cv_server::get_robot_step(const Image &image, Step &step, float scale) {
        auto leds = Detection_list::get_detections(image, robot_threshold, 0).scale(scale).filter(led_profile);
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

    float Cv_server::get_prey_robot_orientation(Image &prey_robot_cam) {
        auto detections = Detection_list::get_detections(prey_robot_cam, 50, 0);
        auto robot_center = detections.filter(mouse_profile);
        if (robot_center.empty()) return 0;
        auto robot_head = detections.filter(prey_robot_head_profile);
        if (robot_head.empty()) return 0;
        return robot_center[0].location.atan(robot_head[0].location);
    }

#define NOLOCATION Location(-1000,-1000)
 enum Screen_image {
        main,
        difference,
        led,
        raw,
        zoom,
        cam0,
        cam1,
        cam2,
        cam3
    };

    void Cv_server::tracking_process() {
        json_cpp::Json_date::set_local_time_zone_offset();
        tracking_running = true;
        puff_state = false;
        Step mouse;
        mouse.location = NOLOCATION;
        Step robot;
        robot.location = NOLOCATION;
        int robot_counter = 0;
        ts.reset();
        int robot_best_cam = -1;
        bool robot_detected;
        bool mouse_detected;
        string screen_text;
        Screen_image screen_image = Screen_image::main;
        double fps = Video::get_fps();
        double time_out = 1.0 / fps * .99;
        Step canonical_step;
        Timer frame_timer(time_out);
        Frame_rate fr;
        fr.filtered_fps = fps;
        fr.filter = .01;
        bool show_occlusions = false;
        int input_counter=0;
        while (tracking_running) {
            PERF_START("WAIT");
            while (!unlimited && !frame_timer.time_out()) this_thread::sleep_for(100us);
            PERF_STOP("WAIT");
            frame_timer.reset();
            PERF_START("CAPTURE");
            auto images = cameras.capture();
            PERF_STOP("CAPTURE");
            PERF_START("COMPOSITE");
            composite.start_composite(images);
            PERF_STOP("COMPOSITE");
            PERF_START("COLOR CONVERSION");
            auto &composite_detection = composite.get_detection_small();
            PERF_STOP("COLOR CONVERSION");
            PERF_START("ROBOT DETECTION");
            if (robot_best_cam == -1) {
                robot_detected = get_robot_step(composite_detection, robot, composite.detection_scale);
            } else {
                robot_detected = get_robot_step(composite.get_detection_small(robot_best_cam), robot, composite.detection_scale);
                if (!robot_detected) {
                    robot_detected = get_robot_step(composite_detection, robot, composite.detection_scale);
                }
            }
            if (robot_detected) {
                auto perspective_adjustment = composite.get_perspective_correction(robot.location, robot_height, robot_best_cam);

                if (!composite.is_transitioning(robot.location) || robot_best_cam == -1) {
                    robot_best_cam = composite.get_best_camera(robot.location);
                }
                robot.location += perspective_adjustment;
                robot_counter = 30;
            } else {
                if (robot_counter) robot_counter--;
                else robot.location = NOLOCATION;
            }
            PERF_STOP("ROBOT DETECTION");
            PERF_START("MOUSE DETECTION");
            PERF_START("DIFF");
            auto diff = composite.get_subtracted_small();
            PERF_STOP("DIFF");
            PERF_START("MOUSE_STEP");
            mouse_detected = get_mouse_step(diff, mouse, robot.location, composite.detection_scale);
            if (mouse_detected) {
                PERF_START("MOUSE_DETECTED");
                composite.start_zoom(mouse.location);
                canonical_step = mouse.convert(cv_space, canonical_space);
                if (waiting_for_prey && canonical_step.location.dist(ENTRANCE) > ENTRANCE_DISTANCE) {
                    waiting_for_prey = false;
                    experiment_client.prey_enter_arena();
                }
                PERF_STOP("MOUSE_DETECTED");
            }
            PERF_STOP("MOUSE_STEP");
            PERF_STOP("MOUSE DETECTION");
            PERF_START("DETECTION_PROCESSING");
            PERF_START("SCREEN");
            auto &composite_image_rgb= composite.get_video();
            PERF_START("SCREEN_ROBOT");
            if (robot_detected) {
                auto color_robot = cv::Scalar({255, 0, 255});
                if (puff_state) {
                    robot.data = "puff";
                    color_robot = cv::Scalar({0, 0, 255});
                    puff_state--;
                } else {
                    robot.data = "";
                }
                composite_image_rgb.circle(robot.location, 5, color_robot, true);
                auto robot_cell_id = composite.map.cells.find(robot.location);
                auto robot_cell_coordinates = composite.map.cells[robot_cell_id].coordinates;
                auto cell_polygon = composite.get_polygon(robot_cell_coordinates);
                composite_image_rgb.polygon(cell_polygon, color_robot);
                composite_image_rgb.arrow(robot.location, to_radians(robot.rotation), 50, color_robot);
            }
            PERF_STOP("SCREEN_ROBOT");
            PERF_START("SCREEN_MOUSE");
            if (mouse_detected) {
                composite_image_rgb.circle(mouse.location, 5, {255, 0, 0}, true);
                auto mouse_cell_id = composite.map.cells.find(mouse.location);
                auto mouse_cell_coordinates = composite.map.cells[mouse_cell_id].coordinates;
                auto cell_polygon = composite.get_polygon(mouse_cell_coordinates);
                composite_image_rgb.polygon(cell_polygon, {255, 0, 0});
            }
            PERF_STOP("SCREEN_MOUSE");
            PERF_STOP("SCREEN");
            PERF_START("MESSAGE");
            if (robot_detected || mouse_detected) {
                thread([this](
                        bool robot_detected,
                        bool mouse_detected,
                        Step robot,
                        Step mouse,
                        float time_stamp,
                        unsigned int frame_number,
                        Tracking_server &tracking_server ) {
                    if (robot_detected) {
                        Predator_data predator_data;
                        predator_data.capture = puff_state ==  PUFF_DURATION;
                        predator_data.best_camera = composite.get_best_camera(robot.location);
                        robot.time_stamp = time_stamp;
                        robot.frame = frame_number;
                        robot.data = predator_data.to_json();
                        tracking_server.send_step(robot.convert(cv_space, canonical_space));
                    }
                    if (mouse_detected) {
                        mouse.time_stamp = time_stamp;
                        mouse.frame = frame_number;
                        mouse.rotation = 0;
                        tracking_server.send_step(mouse.convert(cv_space, canonical_space));
                    }
                },
                       robot_detected,
                       mouse_detected,
                       robot,
                       mouse,
                       ts.to_seconds(),
                       main_video.frame_count,
                       reference_wrapper(tracking_server) ).detach();
            }
            PERF_STOP("MESSAGE");
            PERF_STOP("DETECTION_PROCESSING");
            PERF_START("MOUSE_CUT");
            Image &zoom = composite.get_zoom();
            PERF_STOP("MOUSE_CUT");
            PERF_START("raw_layout");
            auto raw_frame = composite.get_raw_composite();
            PERF_STOP("raw_layout");
            PERF_START("DISPLAY");
            Image screen_frame;
            switch (screen_image) {
                case Screen_image::main :
                    if (show_occlusions) {
                        for (auto &occlusion: occlusions) {
                            composite_image_rgb.circle(occlusion.get().location, 20, {255, 0, 0}, true);
                        }
                    }
                    screen_frame = screen_layout.get_frame(composite_image_rgb, "main", fr.filtered_fps);
                    break;
                case Screen_image::difference :
                    screen_frame = screen_layout.get_frame(diff, "difference", fr.filtered_fps);
                    break;
                case Screen_image::led :
                    {
                        Image small;
                        small.type = Image::Type::gray;
                        cv::resize(composite_detection, small, composite_detection.size()/2);
                        auto small_t = Image(cv::Mat(small > robot_threshold), "");
                        screen_frame = screen_layout.get_frame(small_t, "LEDs", fr.filtered_fps);
                    }
                    break;
                case Screen_image::zoom :
                    screen_frame = screen_layout.get_frame(zoom, "zoom", fr.filtered_fps);
                    break;
                case Screen_image::raw :
                    screen_frame = screen_layout.get_frame(raw_frame, "raw", fr.filtered_fps);
                    break;
                case Screen_image::cam0 :
                    screen_frame = screen_layout.get_frame(images[0], "cam0", fr.filtered_fps);
                    break;
                case Screen_image::cam1 :
                    screen_frame = screen_layout.get_frame(images[1], "cam1", fr.filtered_fps);
                    break;
                case Screen_image::cam2 :
                    screen_frame = screen_layout.get_frame(images[2], "cam2", fr.filtered_fps);
                    break;
                case Screen_image::cam3 :
                    screen_frame = screen_layout.get_frame(images[3], "cam3", fr.filtered_fps);
                    break;
            }
            PERF_STOP("DISPLAY");
            PERF_SCOPE("REST");
            PERF_START("SHOW");
            if (main_video.is_open()) screen_frame.circle({20, 20}, 10, {0, 0, 255}, true);
            cv::imshow("Agent Tracking", screen_frame);
            PERF_STOP("SHOW");
            if (!input_counter) {
                input_counter = 10;
                auto key = cv::waitKey(1);
                switch (key) {
                    case 'C':
                        // start video recording
                        images.save(".");
                        break;
                    case 'H':

                        break;
                    case 'V':
                        // start video recording
                        main_layout.new_episode("","",0,"");
                        main_video.new_video("main");
                        raw_video.new_video("raw");
                        zoom_video.new_video("mouse");
                        break;
                    case 'B':
                        // end video recording
                        main_video.close();
                        raw_video.close();
                        zoom_video.close();
                        zoom_video.split_video(composite.zoom_rectangles, composite.zoom_size);
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
                        composite.set_background(composite.get_detection());
                        break;
                    case 'O':
                        show_occlusions = !show_occlusions;
                        break;
                    case '0' ... '8':
                        screen_image = static_cast<Screen_image>(key-'0');
                        break;
                    case '\t':
                        if (screen_image == Screen_image::cam3)
                            screen_image = Screen_image::main;
                        else
                            screen_image = static_cast<Screen_image>(screen_image + 1);
                        cout << "change_screen_output to " << screen_image << endl;
                        break;
                    case ' ':
                        if (screen_image == Screen_image::main)
                            screen_image = Screen_image::cam3;
                        else
                            screen_image = static_cast<Screen_image>(screen_image - 1);
                        cout << "change_screen_output to " << screen_image << endl;
                        break;
                }
            } else {
                input_counter--;
            }
            fr.new_frame();
            PERF_START("VIDEO");
            if (mouse.location != NOLOCATION) { // starts recording when mouse crosses the door
                PERF_START("LAYOUT");
                auto main_frame = main_layout.get_frame(composite_image_rgb, main_video.frame_count);
                PERF_STOP("LAYOUT");
                main_video.add_frame(main_frame);
                raw_video.add_frame(raw_frame);
                zoom_video.add_frame(zoom);
                // write videos
            }
            PERF_STOP("VIDEO");
        }

    }

    Cv_server::Cv_server(const Camera_configuration &camera_configuration,
                         const std::string &camera_configuration_file,
                         const std::string &background_path,
                         const std::string &video_path,
                         agent_tracking::Tracking_server &tracking_server,
                         Cv_server_experiment_client &experiment_client,
                         bool unlimited):
            tracking_server(tracking_server),
            experiment_client(experiment_client),
            canonical_space(World_implementation::get_from_parameters_name("hexagonal","canonical").space),
            cv_space(World_implementation::get_from_parameters_name("hexagonal","cv").space),
            unlimited(unlimited),
            camera_configuration(camera_configuration),
            cameras(camera_configuration_file, camera_configuration.order.count()),
            composite(camera_configuration),
            main_video(main_layout.size(), Image::rgb),
            raw_video(raw_layout.size(), Image::gray),
            zoom_video(cv::Size(300,300), Image::gray),
            led_profile(Resources::from("profile").key("led").get_resource<Profile>()),
            mouse_profile(Resources::from("profile").key("mouse").get_resource<Profile>()),
            prey_robot_head_profile(Resources::from("profile").key("prey_robot_head").get_resource<Profile>()),
            video_path(video_path)
    {
        experiment_client.cv_server = this;
        Image bg;
        if (file_exists(background_path + "composite.png")){
            bg = Image::read(background_path, "composite.png");
            composite.set_background(bg);
        } else {
            auto images = cameras.capture();
            composite.start_composite(images);
            bg = composite.get_detection();
        }
        auto images = cameras.capture();
        composite.start_composite(images);
        auto &composite_image = composite.get_detection_small();
    }

    void Cv_server_experiment_client::on_episode_started(const string &experiment_name) {
        auto experiment = this->get_experiment(experiment_name);
        std::stringstream ss;
        ss << "/episode_" << std::setw(3) << std::setfill('0') << experiment.episode_count;
        std::string destination_folder = ss.str();
        cv_server->new_episode(experiment.subject_name, experiment.experiment_name, experiment.episode_count, experiment.world_info.occlusions, destination_folder);
    }

    void Cv_server_experiment_client::on_episode_finished() {
        cv_server->end_episode();
    }

    Cv_server_experiment_client::Cv_server_experiment_client() {}

    void Cv_server_experiment_client::on_capture(int) {
        cv_server->puff_state =  PUFF_DURATION;
    }

    void Cv_server_experiment_client::on_experiment_started(const experiment::Start_experiment_response &experiment) {
        cv_server->occlusions = World::get_from_parameters_name(experiment.world.world_configuration,"cv",experiment.world.occlusions).create_cell_group().occluded_cells();
    }

}