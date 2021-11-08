#include <catch.h>
#include <habitat_cv/composite.h>
#include <iostream>
#include <habitat_cv/layout.h>


using namespace std;
using namespace habitat_cv;
using namespace cell_world;


TEST_CASE("layout resize") {
    Layout layout(1000, 1000,Image::Type::rgb);
    Content_resize i0({500,500}, Image::Type::rgb);
    Content_resize i1({500,500}, Image::Type::rgb);
    Content_resize i2({500,500}, Image::Type::rgb);
    Content_resize i3({500,500}, Image::Type::rgb);

    layout.add_place_holder(i0, {500,500});
    layout.add_place_holder(i1, {0,0});
    layout.add_place_holder(i2, {500,0});
    layout.add_place_holder(i3, {0,500});

    auto images = Images::read("../../images/", {"raw_0.png", "raw_1.png", "raw_2.png", "raw_3.png"});

    i0=images.get("raw_0.png").to_rgb();
    i1=images.get("raw_1.png").to_rgb();
    i2=images.get("raw_2.png").to_rgb();
    i3=images.get("raw_3.png").to_rgb();

    layout.get_image().save(".","layout-resize.png");
}

TEST_CASE("layout crop") {
    Layout layout(1000, 1000,Image::Type::rgb);
    Content_crop i0({0.0,0.0}, {500.0,500.0}, Image::Type::rgb);
    Content_crop i1({0.0,0.0}, {500.0,500.0}, Image::Type::rgb);
    Content_crop i2({0.0,0.0}, {500.0,500.0}, Image::Type::rgb);
    Content_crop i3({0.0,0.0}, {500.0,500.0}, Image::Type::rgb);

    layout.add_place_holder(i0, {500,500});
    layout.add_place_holder(i1, {0,0});
    layout.add_place_holder(i2, {500,0});
    layout.add_place_holder(i3, {0,500});

    auto images = Images::read("../../images/", {"raw_0.png", "raw_1.png", "raw_2.png", "raw_3.png"});

    i0=images.get("raw_0.png").to_rgb();
    i1=images.get("raw_1.png").to_rgb();
    i2=images.get("raw_2.png").to_rgb();
    i3=images.get("raw_3.png").to_rgb();

    layout.get_image().save(".","layout-crop.png");
}



TEST_CASE("layout text") {
    Layout layout(300, 600,Image::Type::rgb);
    Content_text text0 ({200,100},Image::Type::rgb,{255,255,255},{50,50,50},.8,0,0);
    Content_text text1 ({200,100},Image::Type::rgb,{255,255,255},{25,25,25},.8, 1,0);
    Content_text text2 ({200,100},Image::Type::rgb,{255,255,255},{0,0,0},.8, 2,0);

    Content_text text3 ({200,100},Image::Type::rgb,{255,255,255},{0,0,0},.8, 0,1);
    Content_text text4 ({200,100},Image::Type::rgb,{255,255,255},{50,50,50},.8, 1,1);
    Content_text text5 ({200,100},Image::Type::rgb,{255,255,255},{25,25,25},.8, 2,1);

    Content_text text6 ({200,100},Image::Type::rgb,{255,255,255},{25,25,25},.8, 0,2);
    Content_text text7 ({200,100},Image::Type::rgb,{255,255,255},{0,0,0},.8, 1,2);
    Content_text text8 ({200,100},Image::Type::rgb,{255,255,255},{50,50,50},.8, 2,2);

    layout.add_place_holder(text0, {0,200});
    layout.add_place_holder(text1, {200,200});
    layout.add_place_holder(text2, {400,200});
    layout.add_place_holder(text3, {0,100});
    layout.add_place_holder(text4, {200,100});
    layout.add_place_holder(text5, {400,100});
    layout.add_place_holder(text6, {0,0});
    layout.add_place_holder(text7, {200,0});
    layout.add_place_holder(text8, {400,0});

    text0 = "text0";
    text1 = "text1";
    text2 = "text2";
    text3 = "text3";
    text4 = "text4";
    text5 = "text5";
    text6 = "text6";
    text7 = "text7";
    text8 = "text8";


    layout.get_image().save(".","layout-text.png");
}


TEST_CASE("complex frame") {
    Layout layout(1125, 1080,Image::Type::rgb);
    Content_crop composite({0.0,45.0}, {1080.0,1030.0}, Image::Type::rgb);
    Content_text date_time ({1080,45},Image::Type::rgb,{255,255,255},{30,30,30},.8,0,1);
    Content_text subject ({540,45},Image::Type::rgb,{255,255,255},{30,30,30},.8,0,1);
    Content_text experiment ({540,45},Image::Type::rgb,{255,255,255},{30,30,30},.8,0,1);
    Content_text episode ({540,45},Image::Type::rgb,{255,255,255},{30,30,30},.8,2,1);
    Content_text frame ({540,45},Image::Type::rgb,{255,255,255},{30,30,30},.8,2,1);


    composite = Image::read(".","composite.png").to_rgb();
    date_time = "  2021-11-02 15:00:05";
    subject = "  Subject: FPP1";
    experiment = "  Experiment: Identifier";
    episode = "Episode: 005  ";
    frame = "Frame: 00005  ";

    layout.add_place_holder(composite, {0,0});
    layout.add_place_holder(date_time, {0,1080});
    layout.add_place_holder(subject, {0,1035});
    layout.add_place_holder(experiment, {0,990});
    layout.add_place_holder(episode, {540,1035});
    layout.add_place_holder(frame, {540,990});


    layout.get_image().save(".","layout-complex.png");
}

struct Main_layout : Layout {
    Main_layout(std::string subject_name, std::string experiment_identifier, int episode_number) :
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

        add_place_holder(composite, {0,0});
        add_place_holder(date_time, {0,1080});
        add_place_holder(subject, {0,1035});
        add_place_holder(experiment, {0,990});
        add_place_holder(episode, {540,1035});
        add_place_holder(frame, {540,990});
    }
    Content_crop composite;
    Content_text date_time ;
    Content_text subject ;
    Content_text experiment;
    Content_text episode;
    Content_text frame;

    Image get_frame(const Image &image, unsigned int frame_count) {
        composite = image;
        frame = "Frame :" + to_string(frame_count) + "  ";
        json_cpp::Json_date d = json_cpp::Json_date::now();
        date_time = "  " + d.to_json();
        return get_image();
    }
};


TEST_CASE("complex frame layout") {
    Main_layout main("FPP1", "robot",1);
    auto composite = Image::read("../../images/","composite.png").to_rgb();
    main.get_frame(composite,588).save("../../images/","main_frame.png");
}
