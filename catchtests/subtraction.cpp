#include <catch.h>
#include <habitat_cv.h>
#include <iostream>

using namespace std;
using namespace habitat_cv;

TEST_CASE("Bg_subtraction") {
    cv::Mat bg = to_gray(cv::imread("../images/camera_0.png"));
    Bg_subtraction bs(bg);
    cv::imshow("sub", bs.subtract(bg));
    CHECK( cv::countNonZero(bs.subtracted) == 0 );
}