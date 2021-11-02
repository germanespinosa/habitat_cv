#include <catch.h>
#include <habitat_cv/composite.h>
#include <iostream>
#include <habitat_cv/image.h>


using namespace std;
using namespace habitat_cv;
using namespace cell_world;


TEST_CASE("clear") {
    Image i(100,100,Image::Type::gray);
    i.save(".","test-clear-gray-before.png");
    i.clear();
    i.save(".","test-clear-gray-after.png");

    Image i2(100,100,Image::Type::rgb);
    i2.save(".","test-clear-rgb-before.png");
    i2.clear();
    i2.save(".","test-clear-rgb-after.png");
}

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
    auto ims2 = ims.to_rgb();
    CHECK (ims2.get("test-i1.png").channels() == 3);
    CHECK (ims2.get("test-i2.png").channels() == 3);
    auto ims3 = ims2.to_gray();
    CHECK (ims3.get("test-i1.png").channels() == 1);
    CHECK (ims3.get("test-i2.png").channels() == 1);
}

TEST_CASE("binary_image"){
    auto bn = Image::read("../../images/","camera_0.png").threshold(128);
    cv::imwrite("test-bn.png",bn);
    Image i (200,200,Image::Type::rgb);
}

TEST_CASE("load_text halign") {
    Image i1(1000, 1000, Image::Type::gray);
    i1.clear();
    i1.text({150.0,200.0},"text1",{180},0.8,0,0);
    i1.text({150.0,500.0},"text2",{180},0.8,0,1);
    i1.text({150.0,800.0},"text3",{180},0.8,0,2);

    i1.text({500.0,200.0},"text4",{180},0.8,1,0);
    i1.text({500.0,500.0},"text5",{180},0.8,1,1);
    i1.text({500.0,800.0},"text6",{180},0.8,1,2);

    i1.text({850.0,200.0},"text7",{180},0.8,2,0);
    i1.text({850.0,500.0},"text8",{180},0.8,2,1);
    i1.text({850.0,800.0},"text9",{180},0.8,2,2);


    i1.circle({150.0,200.0},2,{255});
    i1.circle({150.0,500.0},2,{255});
    i1.circle({150.0,800.0},2,{255});
    i1.circle({500.0,200.0},2,{255});
    i1.circle({500.0,500.0},2,{255});
    i1.circle({500.0,800.0},2,{255});
    i1.circle({850.0,200.0},2,{255});
    i1.circle({850.0,500.0},2,{255});
    i1.circle({850.0,800.0},2,{255});


    i1.save(".","halign.png");
}

