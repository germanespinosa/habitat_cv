#include <catch.h>
#include <detection.h>
#include <iostream>

using namespace habitat_cv;
using namespace std;
using namespace cell_world;

TEST_CASE("Detection"){
    cv::Mat image(500,500,CV_8UC1);
    circle(image,cv::Point(150,100),10,255,cv::FILLED);
    circle(image,cv::Point(100,200),20,255,cv::FILLED);

    circle(image,cv::Point(350,300),10,255,cv::FILLED);
    circle(image,cv::Point(100,400),20,255,cv::FILLED);

    Profile_list pl;
    "[{\"agent_name\":\"small\",\"area_lower_bound\":300,\"area_upper_bound\":350},{\"agent_name\":\"big\",\"area_lower_bound\":1200,\"area_upper_bound\":1300}]" >> pl;

    Detection detection;
    auto detections = detection.get_detections(image,pl);
    CHECK(detections.size() == 4);
    CHECK(detections[0].profile.agent_name == "small");
    CHECK(detections[0].location == Location {150, 100});

    CHECK(detections[1].profile.agent_name == "big");
    CHECK(detections[1].location == Location {100, 200});

    CHECK(detections[2].profile.agent_name == "small");
    CHECK(detections[2].location == Location {350, 300});

    CHECK(detections[3].profile.agent_name == "big");
    CHECK(detections[3].location == Location {100, 400});
}