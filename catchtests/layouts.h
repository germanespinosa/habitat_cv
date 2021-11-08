#pragma once
#include <habitat_cv.h>
struct Main_layout : habitat_cv::Layout {
    Main_layout(std::string subject, std::string experiment, int episode) :
            Layout(1125, 1080,Image::Type::rgb),
            composite({0.0,45.0}, {1080.0,1030.0}, Image::Type::rgb),
            date_time ({1080,45},Image::Type::rgb,{255,255,255},{30,30,30},.8,0,1),
            subject ({540,45},Image::Type::rgb,{255,255,255},{30,30,30},.8,0,1),
            experiment ({540,45},Image::Type::rgb,{255,255,255},{30,30,30},.8,0,1),
            episode ({540,45},Image::Type::rgb,{255,255,255},{30,30,30},.8,2,1),
            frame ({540,45},Image::Type::rgb,{255,255,255},{30,30,30},.8,2,1){
        subject = "  Subject :" + subject_name;
        experiment = "  Experiment :" + experiment_identifier;
        episode = "Episode :" + to_string(episode_number) + "  ";
    }
    habitat_cv::Content_crop composite;
    habitat_cv::Content_text date_time ;
    habitat_cv::Content_text subject ;
    habitat_cv::Content_text experiment;
    habitat_cv::Content_text episode;
    habitat_cv::Content_text frame;
    habitat_cv::Image get_frame(const habitat_cv::Image &, unsigned int frame_count);
};

struct Raw_layout : habitat_cv::Layout {
    Raw_layout();
    std::vector <habitat_cv::Content_resize> panels;
    habitat_cv::Image get_frame(const habitat_cv::Images &);
};