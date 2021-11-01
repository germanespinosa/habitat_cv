#include <catch.h>
#include <habitat_cv/composite.h>
#include <iostream>

using namespace std;
using namespace habitat_cv;
using namespace cell_world;

TEST_CASE("Composite"){
    auto camera_configuration = Resources::from("camera_configuration").key("default").get_resource<Camera_configuration>();
    Composite composite( camera_configuration);
    auto images = Images::read("../../images/",{"camera_0.png","camera_1.png","camera_2.png","camera_3.png"});
    auto comp = composite.get_composite(images);
    auto rgb = comp.to_rgb();
    srand (time(NULL));
    auto cells = composite.world.create_cell_group();
    for (auto i=0;i<30;i++) {
        auto c = cells.random_cell();
        auto coord = composite.get_coordinates(c.location);
        rgb.circle(c.location, 3, {255,255,0});
        rgb.polygon(composite.get_polygon(coord), {255,0,0});
    }
    cv::imwrite("composite.png",comp);
    cv::imwrite("rgb.png",rgb);
//    cv::imshow("test", rgb);
//    cv::waitKey();
}


TEST_CASE("point-coordinates association") {
    auto camera_configuration = Resources::from("camera_configuration").key("default").get_resource<Camera_configuration>();
    Composite composite( camera_configuration);
    auto images = Images::read("../../images/",{"camera_0.png","camera_1.png","camera_2.png","camera_3.png"});
    auto comp = composite.get_composite(images);
    auto rgb = comp.to_rgb();
    srand (time(NULL));
    for (int i=0;i<30; i++) {
        auto point = Location{(double)(rand() % 1080), (double)(rand() % 1080)};
        auto coord = composite.get_coordinates(point);
        rgb.circle(point, 10, {255,0,0}, true);
        rgb.polygon(composite.get_polygon(coord), {255,0,0});
    }
    cv::imwrite("centroid_check.png",rgb);
//    cv::imshow("test", rgb);
//    cv::waitKey();
}


TEST_CASE("arrows") {
    auto camera_configuration = Resources::from("camera_configuration").key("default").get_resource<Camera_configuration>();
    Composite composite( camera_configuration);
    auto images = Images::read("../../images/",{"camera_0.png","camera_1.png","camera_2.png","camera_3.png"});
    auto comp = composite.get_composite(images);
    auto rgb = comp.to_rgb();
    srand (time(NULL));
    for (int i=0;i<30; i++) {
        auto point = Location{(double)(rand() % 1080), (double)(rand() % 1080)};
        auto coord = composite.get_coordinates(point);
        rgb.circle(point, 3, {255,0,0});
        rgb.polygon(composite.get_polygon(coord), {255,0,0});
        double theta = (double)i / 30 * 2 * M_PI;
        rgb.arrow(point, theta, 50.0, {255,0,0});
    }
    cv::imwrite("arrow_check.png",rgb);
//    cv::imshow("test", rgb);
//    cv::waitKey();
}

TEST_CASE("mask") {
    auto wi = Resources::from("world_implementation")
            .key("hexagonal")
            .key("cv").get_resource<World_implementation>();
    auto camera_configuration = Resources::from("camera_configuration").key("default").get_resource<Camera_configuration>();
    Composite composite( camera_configuration);
    auto &space = composite.world.space;
    Image mask(space.transformation.size, space.transformation.size, Image::Type::gray);
    mask.clear();
    Polygon p(space.center, space.shape, space.transformation);
    mask.polygon(p,{255},true);
    mask.save(".","mask.png");
//    cv::imshow("test", rgb);
//    cv::waitKey();
}

