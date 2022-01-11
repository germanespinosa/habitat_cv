#include <catch.h>
#include <cell_world/timer.h>
#include <habitat_cv.h>

using namespace std;
using namespace cell_world;
using namespace habitat_cv;

TEST_CASE("capture") {
    string cam_file = "/habitat/habitat_cv/config/EPIX_IRbest.fmt";
    Camera_array cameras(cam_file, 4);
    cameras.capture();
    Timer t;
    cameras.capture();
    cout << "CAPTURE TIME: " << t.to_seconds() * 1000 << endl;
    cameras.images.save(".", {"raw_0.png", "raw_1.png", "raw_2.png", "raw_3.png"});
    Camera_configuration camera_configuration = Resources::from("camera_configuration").key(
            "default").get_resource<Camera_configuration>();
    Composite large(camera_configuration);
    t.reset();
    large.get_composite(cameras.images);
    cout << "COMPOSITE TIME: " << t.to_seconds() * 1000 << endl;
    large.composite.save(".","composite.png");
    Images mouse_cut;
    for (unsigned int i = 0; i < 4; i++) {
        auto &image = cameras.images[i];
        t.reset();
        auto raw_point = large.get_raw_point(i, {440, 198});
        auto raw_location = image.get_location(raw_point);
        cout << "RAW LOCATION " << i << ": " << raw_location << " TIME: " << t.to_seconds() * 1000 << endl;
        Content_crop cut(raw_location - Location(100, 100), raw_location + Location(100, 100), Image::Type::gray);
        cut = image;
        mouse_cut.emplace_back(cut);
    }
    mouse_cut.save(".", {"mraw_0.png", "mraw_1.png", "mraw_2.png", "mraw_3.png"});
}