#include <catch.h>
#include <habitat_cv/composite.h>
#include <habitat_cv/util.h>
#include <iostream>

using namespace std;
using namespace habitat_cv;


TEST_CASE("Composite"){
    Cameras_associations associations;
    cell_world::Coordinates_list  key_points;
    associations.load("../config/associations.config");
    key_points.load("../config/key_points.config");
    cv::Size size (980,862);
    Camera_order co;
    co.load("../config/camera_order.config");
    Composite composite(size, co, associations);
    auto images = read_images("../images/",{"camera_0.png","camera_1.png","camera_2.png","camera_3.png"});
    cv::Mat comp = composite.get_composite(images, true);
    cv::imwrite("composite.png",comp);
    CHECK(comp.cols == size.width);
    CHECK(comp.rows == size.height);
}

TEST_CASE("get_location") {
    Cameras_associations associations;
    cell_world::Coordinates_list  key_points;
    associations.load("../config/associations.config");
    key_points.load("../config/key_points.config");
    cv::Size size (980,862);
    Camera_order co;
    co.load("../config/camera_order.config");
    Composite composite(size, co, associations);
}


TEST_CASE("get_coordinates") {
    Cameras_associations associations;
    cell_world::Coordinates_list  key_points;
    associations.load("../config/associations.config");
    key_points.load("../config/key_points.config");
    cv::Size size (980,862);
    Camera_order co;
    co.load("../config/camera_order.config");
    Composite composite(size, co, associations);
    auto images = read_images("../images/",{"camera_0.png","camera_1.png","camera_2.png","camera_3.png"});
    cv::Mat comp = composite.get_composite(images);
    cv::Mat rgb;
    cvtColor(comp, rgb, cv::COLOR_GRAY2RGB);
    srand (time(NULL));
    for (int i=0;i<30; i++) {
        auto point = cv::Point{rand() % 980, rand() % 862};
        auto coord = composite.get_coordinates(point);
        composite.draw_circle(rgb, point, 3);
        composite.draw_cell(rgb, coord);
    }
    cv::imwrite("centroid_check.png",comp);
}


