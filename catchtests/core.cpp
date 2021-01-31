#include <catch.h>
#include <core.h>
#include<iostream>


using namespace habitat_cv;
using namespace std;
using namespace cell_world;

TEST_CASE("Cell_association"){
    Cell_association ca;
    "{\"centroid\":{\"x\":1.5,\"y\":2.5}, \"cell_coordinates\":{\"x\":1,\"y\":2}, \"trust\":1}" >> ca;
    Cell_association_list cal;
//    "[{\"centroid\":{\"x\":1.5,\"y\":2.5}, \"cell_coordinates\":{\"x\":1,\"y\":2}, \"trust\":1}]" >> cal;
    cal.load("../../config/associations.config");
    cout << cal.size();
}

TEST_CASE("Camera_order"){
   Camera_order co;
   "[[0,3],[1,2]]" >> co;
   CHECK(co.rows()==2);
   CHECK(co.cols()==2);
   CHECK(co.count()==4);
   CHECK(co.get_camera_coordinates(0)==Coordinates{0,0});
   CHECK(co.get_camera_coordinates(1)==Coordinates{0,1});
   CHECK(co.get_camera_coordinates(2)==Coordinates{1,1});
   CHECK(co.get_camera_coordinates(3)==Coordinates{1,0});
   CHECK(co.get_camera_coordinates(4)==Coordinates{-1,-1});
}

TEST_CASE("Profile"){
    Profile p;
    "{\"agent_name\":\"one\",\"area_lower_bound\":20,\"area_upper_bound\":40}" >> p;
    CHECK(p.match(20));
    CHECK(p.match(30));
    CHECK(p.match(40));
    CHECK_FALSE(p.match(19));
    CHECK_FALSE(p.match(41));

    Profile_list pl;
    "[{\"agent_name\":\"one\",\"area_lower_bound\":20,\"area_upper_bound\":40},{\"agent_name\":\"two\",\"area_lower_bound\":40,\"area_upper_bound\":60}]" >> pl;

    CHECK(pl.match(20).size() == 1);
    CHECK(pl.match(20)[0].agent_name == "one");
    CHECK(pl.match(40).size() == 2);
    CHECK(pl.match(40)[0].agent_name == "one");
    CHECK(pl.match(40)[1].agent_name == "two");
    CHECK(pl.match(41).size() == 1);
    CHECK(pl.match(41)[0].agent_name == "two");
}
