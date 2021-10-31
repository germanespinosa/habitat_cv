#include <catch.h>
#include <habitat_cv/composite.h>
#include <iostream>

using namespace std;
using namespace habitat_cv;
using namespace cell_world;

TEST_CASE("gray_image save") {
    Image i(100,100,Image::Type::gray);
    CHECK(i.channels()==1);
    i.save(".","test-gray.png");
    auto i2 = Image::read(".","test-gray.png");
    CHECK(i2.channels()==1);
}


TEST_CASE("rgb_image save") {
    Image i(100,100,Image::Type::rgb);
    CHECK(i.channels()==3);
    i.save(".","test-rgb.png");
    auto i2 = Image::read(".","test-rgb.png");
    CHECK(i2.channels()==3);
}

TEST_CASE("load_multiple") {
    Image i1(100,100,Image::Type::gray);
    Image i2(100,100,Image::Type::rgb);
    i1.save(".","test-i1.png");
    i2.save(".","test-i2.png");
    auto ims = Images::read (".",".png");
    CHECK (ims.get("test-i1.png").channels() == 1);
    CHECK (ims.get("test-i2.png").channels() == 3);
    ims.to_rgb().save(".");
    auto ims2 = Images::read (".",".png");
    CHECK (ims2.get("test-i1.png").channels() == 3);
    CHECK (ims2.get("test-i2.png").channels() == 3);
    ims2.to_gray().save(".");
    auto ims3 = Images::read (".",".png");
    CHECK (ims3.get("test-i1.png").channels() == 1);
    CHECK (ims3.get("test-i2.png").channels() == 1);
}
