#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <stdexcept>

#include "ScenarioEngine.hpp"
#include "ScenarioReader.hpp"
#include "ControllerUDPDriver.hpp"
#include "pugixml.hpp"
#include "simple_expr.h"

using namespace roadmanager;
using namespace scenarioengine;

#define TRIG_ERR_MARGIN 0.001
#define LOG_TO_CONSOLE 0

TEST(DistanceTest, CalcDistanceVariations)
{
    double dist = 0.0;
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/curve_r100.xodr");
    OpenDrive* odr = Position::GetOpenDrive();

    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 1);

    Position pos0 = Position(0, -1, 400.0, 0);
    pos0.SetHeading(0.0);

    // another point some meters ahead, still on the same straight segment
    Position pos1 = Position(0, -1, 410.0, 0);
    pos1.SetHeading(0.0);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, dist), 0);
    EXPECT_NEAR(dist, 10.0, 1e-5);

    // same point but measure now in road coordinates
    Object obj0(Object::Type::VEHICLE);
    obj0.boundingbox_.center_ = { 1.0, 0.0, 0.0 };
    obj0.boundingbox_.dimensions_ = { 2.0, 2.0, 2.0 };
    obj0.pos_ = pos0;

    Object obj1(Object::Type::VEHICLE);
    obj1.pos_ = pos1;
    obj1.boundingbox_.center_ = { 1.0, 0.0, 0.0 };
    obj1.boundingbox_.dimensions_ = { 2.0, 2.0, 2.0 };

    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, true, dist), 0);
    EXPECT_NEAR(dist, 8.0, 1e-5);

    // Modify boundingbox center
    obj1.boundingbox_.center_ = { 1.5, 0.0, 0.0 };
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, true, dist), 0);
    EXPECT_NEAR(dist, 8.5, 1e-5);

    // Move object to curve
    obj0.pos_.SetLanePos(0, -1, 510.0, 0.0);
    obj1.pos_.SetLanePos(0, -1, 550.0, 2.5);

    // ref point to ref point (no freespace)
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, false, dist), 0);
    EXPECT_NEAR(dist, 40.0, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, false, dist), 0);
    EXPECT_NEAR(dist, 2.5, 1e-5);

    // freespace
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, true, dist), 0);
    EXPECT_NEAR(dist, 38.510726, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, true, dist), 0);
    EXPECT_NEAR(dist, 0.468766, 1e-5);

    // freespace with overlap
    obj0.pos_.SetAlignMode(Position::ALIGN_MODE::ALIGN_HARD);
    obj0.pos_.SetLanePos(0, -1, 549.0, 0.0);
    obj1.pos_.SetAlignMode(Position::ALIGN_MODE::ALIGN_HARD);
    obj1.pos_.SetLanePos(0, -1, 550.0, 0.0);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, true, dist), 0);
    EXPECT_NEAR(dist, 0.0, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, true, dist), 0);
    EXPECT_NEAR(dist, 0.0, 1e-5);

    // Create a lateral gap
    obj1.pos_.SetLanePos(0, -1, 550.0, 2.5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, false, dist), 0);
    EXPECT_NEAR(dist, 2.5, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, true, dist), 0);
    EXPECT_NEAR(dist, 0.4687658, 1e-5);

    // Move to straight segment and do same measurement
    obj0.pos_.SetLanePos(0, -1, 50.0, 0.0);
    obj1.pos_.SetLanePos(0, -1, 51.0, 2.5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, false, dist), 0);
    EXPECT_NEAR(dist, 2.5, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, true, dist), 0);
    EXPECT_NEAR(dist, 0.5, 1e-5);

    // Move second object behind first one
    obj0.pos_.SetLanePos(0, -1, 50.0, 0.0);
    obj1.pos_.SetLanePos(0, -1, 49.0, 2.5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, false, dist), 0);
    EXPECT_NEAR(dist, 2.5, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, true, dist), 0);
    EXPECT_NEAR(dist, 0.5, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, false, dist), 0);
    EXPECT_NEAR(dist, -1.0, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, true, dist), 0);
    EXPECT_NEAR(dist, 0.0, 1e-5);

    // Move to other side of road, sign of relative distances should be the same
    obj0.pos_.SetLanePos(0, 1, 49.0, 0.0);
    obj0.pos_.SetHeadingRelative(M_PI);
    obj1.pos_.SetLanePos(0, 1, 50.0, -2.5);
    obj1.pos_.SetHeadingRelative(M_PI);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, false, dist), 0);
    EXPECT_NEAR(dist, 2.5, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, true, dist), 0);
    EXPECT_NEAR(dist, 0.5, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, false, dist), 0);
    EXPECT_NEAR(dist, -1.0, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, true, dist), 0);
    EXPECT_NEAR(dist, 0.0, 1e-5);
}

TEST(DistanceTest, CalcDistancePoint)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/curve_r100.xodr");
    OpenDrive* odr = Position::GetOpenDrive();

    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 1);

    Position pos0 = Position(0, -1, 400.0, 0);
    pos0.SetHeading(0.0);

    Object obj0(Object::Type::VEHICLE);
    obj0.boundingbox_.center_ = { 1.0, 0.0, 0.0 };
    obj0.boundingbox_.dimensions_ = { 2.0, 2.0, 2.0 };
    obj0.pos_ = pos0;

    // Measure from X, Y point to object in road coordinates
    double latDist = 0.0;
    double longDist = 0.0;

    ASSERT_EQ(obj0.FreeSpaceDistancePointRoadLane(pos0.GetX() + 20.0, pos0.GetY(), &latDist, &longDist, CoordinateSystem::CS_ROAD), 0);
    EXPECT_NEAR(longDist, 18.0, 1e-5);

    ASSERT_EQ(obj0.FreeSpaceDistancePointRoadLane(pos0.GetX() - 20.0, pos0.GetY(), &latDist, &longDist, CoordinateSystem::CS_ROAD), 0);
    EXPECT_NEAR(longDist, -20.0, 1e-5);

    obj0.boundingbox_.dimensions_ = { 2.0, 5.0, 2.0 };
    ASSERT_EQ(obj0.FreeSpaceDistancePointRoadLane(pos0.GetX() - 20.0, pos0.GetY(), &latDist, &longDist, CoordinateSystem::CS_ROAD), 0);
    EXPECT_NEAR(longDist, -18.5, 1e-5);

    obj0.boundingbox_.center_ = { 2.0, 4.0, 0.0 };
    ASSERT_EQ(obj0.FreeSpaceDistancePointRoadLane(pos0.GetX() - 20.0, pos0.GetY(), &latDist, &longDist, CoordinateSystem::CS_ROAD), 0);
    EXPECT_NEAR(longDist, -19.5, 1e-5);

    obj0.boundingbox_.center_ = { 2.0, 4.0, 0.0 };
    Position tmpPos(0, 600, obj0.pos_.GetT());
    EXPECT_NEAR(tmpPos.GetX(), 585.438756, 1e-5);
    EXPECT_NEAR(tmpPos.GetY(), 45.140405, 1e-5);
    ASSERT_EQ(obj0.FreeSpaceDistancePointRoadLane(tmpPos.GetX(), tmpPos.GetY(), &latDist, &longDist, CoordinateSystem::CS_ROAD), 0);
    EXPECT_NEAR(longDist, 195.5, 1e-5);
    EXPECT_NEAR(latDist, 3.0, 1e-5);
}

TEST(DistanceTest, CalcDistancePointAcrossIntersection)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/fabriksgatan.xodr");
    OpenDrive* odr = Position::GetOpenDrive();

    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 16);

    Position pos0 = Position(0, 1, 10.0, 0);
    pos0.SetHeading(0.0);

    Object obj0(Object::Type::VEHICLE);
    obj0.boundingbox_.center_ = { 1.0, 0.0, 0.0 };
    obj0.boundingbox_.dimensions_ = { 2.0, 2.0, 2.0 };
    obj0.pos_ = pos0;

    // another point some meters ahead, still on the same straight segment
    Position pos1 = Position(2, 1, 290.0, 0);

    // Measure from X, Y point to object in road coordinates
    double latDist = 0.0;
    double longDist = 0.0;

    ASSERT_EQ(obj0.FreeSpaceDistancePointRoadLane(pos1.GetX(), pos1.GetY(), &latDist, &longDist, CoordinateSystem::CS_ROAD), 0);
    EXPECT_NEAR(longDist, -38.587016, 1e-5);
    EXPECT_NEAR(latDist, 0.221272, 1e-5);

}

TEST(DistanceTest, CalcEntityDistanceFreespace)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/straight_500m.xodr");
    OpenDrive* odr = Position::GetOpenDrive();

    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 1);

    Object obj0(Object::Type::VEHICLE);
    obj0.boundingbox_.center_ = { 1.5, 0.0, 0.0 };
    obj0.boundingbox_.dimensions_ = { 2.0, 5.0, 2.0 };
    obj0.pos_.SetLanePos(1, -1, 20.0, 0);
    obj0.pos_.SetHeading(0.0);

    Object obj1(Object::Type::VEHICLE);
    obj1.boundingbox_.center_ = { 1.5, 0.0, 0.0 };
    obj1.boundingbox_.dimensions_ = { 2.0, 5.0, 2.0 };
    obj1.pos_.SetLanePos(1, -1, 30.0, 0);
    obj1.pos_.SetHeading(0.0);

    // Measure from X, Y point to object in cartesian coordinates
    double latDist = 0.0;
    double longDist = 0.0;
    double dist = 0.0;
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, 0, 0), false);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, &latDist, &longDist), false);
    EXPECT_NEAR(latDist, 0.0, 1e-5);
    EXPECT_NEAR(longDist, 5.0, 1e-5);
    EXPECT_NEAR(dist = obj0.FreeSpaceDistance(&obj1, &latDist, &longDist), 5.0, 1e-3);

    obj1.pos_.SetLanePos(1, -1, 15.1, 1.9);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, 0, 0), true);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, &latDist, &longDist), true);
    EXPECT_NEAR(latDist, 0.0, 1e-5);
    EXPECT_NEAR(longDist, 0.0, 1e-5);
    EXPECT_NEAR(dist = obj0.FreeSpaceDistance(&obj1, &latDist, &longDist), 0.0, 1e-3);

    obj1.pos_.SetLanePos(1, -1, 15.1, 2.9);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, 0, 0), false);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, &latDist, &longDist), false);
    EXPECT_NEAR(latDist, 0.9, 1e-5);
    EXPECT_NEAR(longDist, 0.0, 1e-5);
    EXPECT_NEAR(dist = obj0.FreeSpaceDistance(&obj1, &latDist, &longDist), 0.9, 1e-3);

    obj1.pos_.SetLanePos(1, -1, 10.0, 0.0);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, 0, 0), false);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, &latDist, &longDist), false);
    EXPECT_NEAR(latDist, 0.0, 1e-5);
    EXPECT_NEAR(longDist, -5.0, 1e-5);
    EXPECT_NEAR(dist = obj0.FreeSpaceDistance(&obj1, &latDist, &longDist), 5.0, 1e-3);

    obj1.pos_.SetLanePos(1, -1, 10.0, 0.0);
    obj1.pos_.SetHeadingRelative(M_PI);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, 0, 0), false);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, &latDist, &longDist), false);
    EXPECT_NEAR(latDist, 0.0, 1e-5);
    EXPECT_NEAR(longDist, -8.0, 1e-5);
    EXPECT_NEAR(dist = obj0.FreeSpaceDistance(&obj1, &latDist, &longDist), 8.0, 1e-3);

    obj1.pos_.SetLanePos(1, -1, 30.0, 5.0);
    obj1.pos_.SetHeadingRelative(0.5);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, 0, 0), false);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, &latDist, &longDist), false);
    EXPECT_NEAR(latDist, 2.64299, 1e-5);
    EXPECT_NEAR(longDist, 4.64299, 1e-5);
    EXPECT_NEAR(dist = obj0.FreeSpaceDistance(&obj1, &latDist, &longDist), 6.183198, 1e-3);

    obj1.pos_.SetHeadingRelative(-0.5);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, &latDist, &longDist), false);
    EXPECT_NEAR(latDist, 1.204715, 1e-5);
    EXPECT_NEAR(longDist, 4.64299, 1e-5);
    EXPECT_NEAR(dist = obj0.FreeSpaceDistance(&obj1, &latDist, &longDist), 5.876278, 1e-3);
}

TEST(TrajectoryTest, EnsureContinuation)
{
    double dt = 0.01;
    ScenarioEngine *se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/trajectory-continuity.xosc");
    ASSERT_NE(se, nullptr);

    for (int i = 0; i < (int)(1.0/dt); i++)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities.object_[0]->pos_.GetX(), 4.95, 1e-5);
    ASSERT_NEAR(se->entities.object_[0]->pos_.GetY(), -1.535, 1e-5);

    for (int i = 0; i < (int)(2.0 / dt); i++)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities.object_[0]->pos_.GetX(), 14.92759, 1e-5);
    ASSERT_NEAR(se->entities.object_[0]->pos_.GetY(), -1.18333, 1e-5);

    for (int i = 0; i < (int)(1.5 / dt); i++)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities.object_[0]->pos_.GetX(), 21.31489, 1e-5);
    ASSERT_NEAR(se->entities.object_[0]->pos_.GetY(), 2.56606, 1e-5);
}

TEST(ExpressionTest, EnsureResult)
{
    ASSERT_FLOAT_EQ(eval_expr("1 + 1"), 2.0f);
    ASSERT_FLOAT_EQ(eval_expr("5 * 10 + 1"), 51.0f);
    ASSERT_FLOAT_EQ(eval_expr("5 * (10 + 1)"), 55.0f);
    ASSERT_FLOAT_EQ(eval_expr("15/3.5"), 15.0f / 3.5f);
    ASSERT_FLOAT_EQ(eval_expr("15 % 6"), 3.0f);
    ASSERT_FLOAT_EQ(eval_expr("-15 % 6"), -3.0f);
    ASSERT_FLOAT_EQ(eval_expr("1 == 1"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("1 == 2"), 0.0f);
    ASSERT_FLOAT_EQ(eval_expr("(4 == 4) && (10 == 10)"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("4 == 4 && 10 == 10"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("4 == 4 && 9 < 10"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("4 == 4 || 11 == 10"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("4 == 3 || 9 < 8"), 0.0f);
    ASSERT_FLOAT_EQ(eval_expr("ceil(11.1) == 12"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("round(11.1) == 11"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("floor(11.9) == 11"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("ceil(-11.1) == -11"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("round(-11.1) == -11"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("floor(-11.9) == -12"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("pow(2,3)"), 8.0f);
    ASSERT_FLOAT_EQ(eval_expr("2**3"), 8.0f);
}

TEST(OptionsTest, TestOptionHandling)
{
    SE_Options opt;

    // define arguments
    opt.AddOption("osc_file", "Scenario file", "filename");
    opt.AddOption("odr_file", "Roadnetwork file", "filename", "default_road.xodr");
    opt.AddOption("option2", "Some value", "value");
    opt.AddOption("option3", "Some rate", "rate", "55");
    opt.AddOption("option4", "Some temp", "temp");
    opt.AddOption("option5", "Some speed", "speed");
    opt.AddOption("window", "Visualize the scenario");
    // opt.PrintUsage();

    // set arguments
    const char* args[] = {
        "my_app",
        "--osc_file",
        "my_scenario.xosc",
        "--odr_file",
        "my_road.xodr",
        "--window",
        "--option2",
        "option2Value",
        "--option2",
        "option2Value2",
        "--option3",
        "--option4"
    };
    int argc = sizeof(args) / sizeof(char*);
    char** argv;
    argv = (char**)malloc(argc * sizeof(char*));
    for (int i = 0; i < argc; i++)
    {
        argv[i] = (char*)malloc(strlen(args[i]) + 1);
        strncpy(argv[i], args[i], strlen(args[i]) + 1);
    }

    ASSERT_EQ(opt.ParseArgs(&argc, argv), -1);

    ASSERT_EQ(opt.GetOptionSet("no_arg"), false);
    ASSERT_EQ(opt.GetOptionSet("osc_file"), true);
    ASSERT_EQ(opt.GetOptionSet("window"), true);
    ASSERT_EQ(opt.GetOptionSet("option2"), true);
    ASSERT_EQ(opt.GetOptionSet("option3"), true);
    ASSERT_EQ(opt.GetOptionSet("option4"), false);
    ASSERT_EQ(opt.GetOptionSet("option5"), false);
    ASSERT_EQ(opt.GetOptionArg("window"), "");
    ASSERT_EQ(opt.GetOptionArg("osc_file"), "my_scenario.xosc");
    ASSERT_EQ(opt.GetOptionArg("odr_file"), "my_road.xodr");
    ASSERT_EQ(opt.GetOptionArg("option2"), "option2Value");
    ASSERT_EQ(opt.GetOptionArg("option2", 1), "option2Value2");
    ASSERT_EQ(opt.GetOptionArg("option3"), "55");

    // test without last argument, should return OK
    argc = sizeof(args-1) / sizeof(char*);
    ASSERT_EQ(opt.ParseArgs(&argc, argv), 0);

    // Clean up
    for (int i = 0; i < argc; i++)
    {
        delete argv[i];
    }
    delete argv;
}

TEST(ParameterTest, ResolveParameterTest)
{
    Parameters params;

    params.parameterDeclarations_.Parameter.push_back( { "speed", OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE, {0, 5.0, "5.0", false} });
    params.parameterDeclarations_.Parameter.push_back({ "acc", OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE, {0, 3.0, "3.0", false} });
    params.parameterDeclarations_.Parameter.push_back({ "turnsignal", OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE, {0, 0.0, "true", true} });

    ASSERT_EQ(params.ResolveParametersInString("$speed + 1.0"), "5.0 + 1.0");
    ASSERT_EQ(params.ResolveParametersInString("$speed $acc"), "5.0 3.0");
    ASSERT_EQ(params.ResolveParametersInString("$speed $acc "), "5.0 3.0 ");
    ASSERT_EQ(params.ResolveParametersInString(" $speed $acc "), " 5.0 3.0 ");
    ASSERT_EQ(params.ResolveParametersInString(" speed $acc "), " speed 3.0 ");
    ASSERT_EQ(params.ResolveParametersInString(" $turnsignal "), " true ");
}

TEST(ParameterTest, ParseParameterTest)
{
    // Create parameter declarations
    pugi::xml_document xml_doc;
    pugi::xml_node paramDeclsNode = xml_doc.append_child("paramDeclsNode");

    pugi::xml_node paramDeclNode0 = paramDeclsNode.append_child("paramDeclNode0");
    paramDeclNode0.append_attribute("name") = "param0";
    paramDeclNode0.append_attribute("parameterType") = "double";
    paramDeclNode0.append_attribute("value") = "17.0";

    pugi::xml_node paramDeclNode1 = paramDeclsNode.append_child("paramDeclNode1");
    paramDeclNode1.append_attribute("name") = "param1";
    paramDeclNode1.append_attribute("parameterType") = "boolean";
    paramDeclNode1.append_attribute("value") = "true";

    Parameters params;
    params.addParameterDeclarations(paramDeclsNode);

    // Create an XML element with attributes referring to parameters
    pugi::xml_node someNode0 = xml_doc.append_child("someNode0");
    someNode0.append_attribute("speed") = "5.1";
    someNode0.append_attribute("acc") = "$param0";
    someNode0.append_attribute("attr2") = "${$param0 + 0.5}";
    someNode0.append_attribute("attr3") = "${$param1 && (1==1)}";
    someNode0.append_attribute("attr4") = "${$param1 || (0==1)}";
    someNode0.append_attribute("attr5") = "${$param1 and (0==1)}";
    someNode0.append_attribute("attr6") = "${$param1 and not(0==1)}";
    someNode0.append_attribute("attr7") = "${$param1 and not (0==1)}";
    someNode0.append_attribute("attr8") = "${$param1 and (26 == $param0 + 9)}";
    someNode0.append_attribute("attr9") = "${(2 + ($param0- 9)/2)/3}";

    // verify correct parameter lookup
    ASSERT_EQ(params.ReadAttribute(someNode0, "speed", false), "5.1");
    ASSERT_EQ(params.ReadAttribute(someNode0, "acc", false), "17.0");
    ASSERT_EQ(params.ReadAttribute(someNode0, "attr2", false), "17.500000");
    ASSERT_EQ(params.ReadAttribute(someNode0, "attr3", false), "1.000000");
    ASSERT_EQ(params.ReadAttribute(someNode0, "attr4", false), "1.000000");
    ASSERT_EQ(params.ReadAttribute(someNode0, "attr5", false), "0.000000");
    ASSERT_EQ(params.ReadAttribute(someNode0, "attr6", false), "1.000000");
    ASSERT_EQ(params.ReadAttribute(someNode0, "attr7", false), "1.000000");
    ASSERT_EQ(params.ReadAttribute(someNode0, "attr8", false), "1.000000");
    ASSERT_EQ(params.ReadAttribute(someNode0, "attr9", false), "2.000000");
}

TEST(JunctionTest, JunctionSelectorTest)
{
    double dt = 0.01;

    double angles[] = { 3 * M_PI_2, -M_PI_2, 0.0, M_PI_2 };
    int roadIds[] = { 1, 1, 2, 3 };
    double durations[] = { 2.5, 2.5, 2.6, 2.8 };  // Make sure car gets gets out of the intersection

    for (int i = 0; i < sizeof(angles) / sizeof(double); i++)
    {
        ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/junction-selector.xosc");
        se->step(0.0);
        se->prepareGroundTruth(0.0);
        ASSERT_NE(se, nullptr);

        // Turn always right
        se->entities.object_[0]->SetJunctionSelectorStrategy(roadmanager::Junction::JunctionStrategyType::SELECTOR_ANGLE);
        se->entities.object_[0]->SetJunctionSelectorAngle(angles[i]);
        while (se->getSimulationTime() < durations[i] && se->GetQuitFlag() != true)
        {
            se->step(dt);
            se->prepareGroundTruth(dt);
        }
        ASSERT_EQ(se->entities.object_[0]->pos_.GetTrackId(), roadIds[i]);
        delete se;
    }
}

TEST(ConditionTest, CollisionTest)
{
    double dt = 0.01;

    double timestamps[] = { 5.23, 5.24, 6.23, 6.25, 7.09, 8.77 };

    // Initialize the scenario and disable interactive controller
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/test-collision-detection.xosc", true);
    se->step(0.0);
    se->prepareGroundTruth(0.0);
    ASSERT_NE(se, nullptr);

    while (se->getSimulationTime() < timestamps[0] && se->GetQuitFlag() != true)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[1]), false);
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[2]), false);

    while (se->getSimulationTime() < timestamps[1] && se->GetQuitFlag() != true)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[1]), false);
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[2]), true);

    while (se->getSimulationTime() < timestamps[2] && se->GetQuitFlag() != true)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[1]), false);
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[2]), true);

    while (se->getSimulationTime() < timestamps[3] && se->GetQuitFlag() != true)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[1]), true);
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[2]), true);

    while (se->getSimulationTime() < timestamps[4] && se->GetQuitFlag() != true)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[1]), true);
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[2]), false);

    while (se->getSimulationTime() < timestamps[5] && se->GetQuitFlag() != true)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[1]), false);
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[2]), false);

    delete se;
}

TEST(ControllerTest, UDPDriverModelTestAsynchronous)
{
    double dt = 0.01;

    ScenarioEngine* se = new ScenarioEngine("../../../scripts/udp-driver/two_cars_in_open_space.xosc");
    ASSERT_NE(se, nullptr);
    ASSERT_EQ(se->entities.object_.size(), 2);

    // Replace controllers
    for (int i = 0; i < 2; i++)
    {
        scenarioengine::Controller::InitArgs args;
        args.name = "UDPDriverModel Controller";
        args.type = ControllerUDPDriver::GetTypeNameStatic();
        args.parameters = 0;
        args.gateway = se->getScenarioGateway();
        args.properties = new OSCProperties();
        OSCProperties::Property property;
        property.name_ = "port";
        property.value_ = std::to_string(0);
        args.properties->property_.push_back(property);
        property.name_ = "basePort";
        property.value_ = std::to_string(61900);
        args.properties->property_.push_back(property);
        property.name_ = "inputMode";
        property.value_ = "vehicleStateXYZHPR";
        args.properties->property_.push_back(property);
        ControllerUDPDriver* controller = (ControllerUDPDriver*)InstantiateControllerUDPDriver(&args);

        delete se->entities.object_[i]->controller_;
        delete args.properties;

        controller->Assign(se->entities.object_[i]);
        se->scenarioReader->controller_[i] = controller;
        se->entities.object_[i]->controller_ = controller;
    }

    // assign controllers
    se->step(dt);

    // stimulate driver input
    UDPClient* udpClient= new UDPClient(61900, "127.0.0.1");

    ControllerUDPDriver::DMMessage msg;

    msg.header.frameNumber = 0;
    msg.header.version = 1;
    msg.header.objectId = 0;
    msg.header.inputMode = static_cast<int>(ControllerUDPDriver::InputMode::VEHICLE_STATE_XYH);

    msg.message.stateXYZHPR.h = 0.3;
    msg.message.stateXYZHPR.x = 20.0;
    msg.message.stateXYZHPR.y = 30.0;

    udpClient->Send((char*)&msg, sizeof(msg));

    // Make sure last message is applied
    msg.message.stateXYZHPR.y = 40;
    udpClient->Send((char*)&msg, sizeof(msg));

    // read messages and report updated states
    se->step(dt);
    // another step for scenarioengine to fetch and apply updated states
    se->step(dt);

    EXPECT_DOUBLE_EQ(se->entities.object_[0]->pos_.GetY(), 40.0);

    delete se;
    delete udpClient;
}

TEST(ControllerTest, UDPDriverModelTestSynchronous)
{
    double dt = 0.01;

    ScenarioEngine* se = new ScenarioEngine("../../../scripts/udp-driver/two_cars_in_open_space.xosc");
    ASSERT_NE(se, nullptr);
    ASSERT_EQ(se->entities.object_.size(), 2);

    // Replace controllers
    for (int i = 0; i < 2; i++)
    {
        scenarioengine::Controller::InitArgs args;
        args.name = "UDPDriverModel Controller";
        args.type = ControllerUDPDriver::GetTypeNameStatic();
        args.parameters = 0;
        args.gateway = se->getScenarioGateway();
        args.properties = new OSCProperties();
        OSCProperties::Property property;
        property.name_ = "execMode";
        property.value_ = "synchronous";
        args.properties->property_.push_back(property);
        property.name_ = "port";
        property.value_ = std::to_string(0);
        args.properties->property_.push_back(property);
        property.name_ = "basePort";
        property.value_ = std::to_string(61910);
        args.properties->property_.push_back(property);
        property.name_ = "inputMode";
        property.value_ = "vehicleStateXYZHPR";
        args.properties->property_.push_back(property);
        property.name_ = "timoutMs";
        property.value_ = std::to_string(500);
        args.properties->property_.push_back(property);
        ControllerUDPDriver* controller = (ControllerUDPDriver*)InstantiateControllerUDPDriver(&args);

        delete se->entities.object_[i]->controller_;
        delete args.properties;

        controller->Assign(se->entities.object_[i]);
        se->scenarioReader->controller_[i] = controller;
        se->entities.object_[i]->controller_ = controller;
    }

    // assign controllers
    se->step(dt);

    // stimulate driver input
    UDPClient* udpClient = new UDPClient(61910, "127.0.0.1");

    ControllerUDPDriver::DMMessage msg;

    msg.header.frameNumber = 0;
    msg.header.version = 1;
    msg.header.objectId = 0;
    msg.header.inputMode = static_cast<int>(ControllerUDPDriver::InputMode::VEHICLE_STATE_XYZHPR);

    msg.message.stateXYZHPR.h = 0.3;
    msg.message.stateXYZHPR.x = 20.0;
    msg.message.stateXYZHPR.y = 30.0;

    udpClient->Send((char*)&msg, sizeof(msg));

    // Put another message on queue of first vehicle
    msg.header.frameNumber++;
    msg.message.stateXYZHPR.y = 40;
    udpClient->Send((char*)&msg, sizeof(msg));

    // read messages and report updated states
    se->step(dt);

    // another step for scenarioengine to fetch and apply updated states
    se->step(dt);

    // In synchronous mode one message is consumed each time step
    // Expect the first message to be applied, the second has not
    // yet been processed
    EXPECT_DOUBLE_EQ(se->entities.object_[0]->pos_.GetY(), 30.0);

    // another step for scenarioengine to fetch and apply the second message
    se->step(dt);
    EXPECT_DOUBLE_EQ(se->entities.object_[0]->pos_.GetY(), 40.0);

    // second vehicle has not been updated (no message sent)
    se->step(dt);
    EXPECT_DOUBLE_EQ(se->entities.object_[1]->pos_.GetY(), 6.5);

    // Create a sender for second vehicle as well
    UDPClient* udpClient2 = new UDPClient(61911, "127.0.0.1");

    msg.header.frameNumber = 0;
    msg.header.version = 1;
    msg.header.objectId = 1;
    msg.header.inputMode = static_cast<int>(ControllerUDPDriver::InputMode::VEHICLE_STATE_XYZHPR);

    msg.message.stateXYZHPR.h = 0.3;
    msg.message.stateXYZHPR.x = 90.0;
    msg.message.stateXYZHPR.y = -10.0;

    udpClient2->Send((char*)&msg, sizeof(msg));

    msg.header.frameNumber = 2;
    msg.header.objectId = 0;
    msg.message.stateXYZHPR.x = 150.0;
    udpClient->Send((char*)&msg, sizeof(msg));

    se->step(dt);
    se->step(dt);
    EXPECT_DOUBLE_EQ(se->entities.object_[0]->pos_.GetX(), 150.0);
    EXPECT_DOUBLE_EQ(se->entities.object_[1]->pos_.GetX(), 90.0);
    EXPECT_DOUBLE_EQ(se->entities.object_[1]->pos_.GetY(), -10.0);

    delete se;
    delete udpClient;
}


TEST(EnvironmentTest, Basic)
{
    OSCEnvironment environment;
    OSCEnvironment new_environment;

    new_environment.SetAtmosphericPressure(100000);
    environment.UpdateEnvironment(&new_environment);
    EXPECT_EQ(new_environment.GetAtmosphericPressure(),100000);
    EXPECT_EQ(new_environment.GetAtmosphericPressure(),environment.GetAtmosphericPressure());
    EXPECT_TRUE(new_environment.IsAtmosphericPressure());
    EXPECT_TRUE(environment.IsAtmosphericPressure());

    new_environment.SetTemperature(293);
    environment.UpdateEnvironment(&new_environment);
    EXPECT_EQ(new_environment.GetTemperature(),293);
    EXPECT_EQ(new_environment.GetTemperature(),environment.GetTemperature());
    EXPECT_TRUE(new_environment.IsTemperature());
    EXPECT_TRUE(environment.IsTemperature());

    new_environment.SetCloudState(scenarioengine::CloudState::HEAVILY_CLOUDY);
    environment.UpdateEnvironment(&new_environment);
    EXPECT_EQ(new_environment.GetCloudState(),scenarioengine::CloudState::HEAVILY_CLOUDY);
    EXPECT_EQ(new_environment.GetCloudState(),environment.GetCloudState());
    EXPECT_TRUE(new_environment.IsCloudState());
    EXPECT_TRUE(environment.IsCloudState());
    
    new_environment.SetFog(1000);
    environment.UpdateEnvironment(&new_environment);
    EXPECT_EQ(new_environment.GetFog()->visibility_range,1000);
    EXPECT_EQ(new_environment.GetFog()->visibility_range,environment.GetFog()->visibility_range);
    EXPECT_TRUE(new_environment.IsFog());
    EXPECT_TRUE(environment.IsFog());

    new_environment.SetPrecipitation(0.1,scenarioengine::PrecipitationType::SNOW);
    environment.UpdateEnvironment(&new_environment);
    EXPECT_EQ(new_environment.GetPrecipitation()->precipitationintensity,0.1);
    EXPECT_EQ(new_environment.GetPrecipitation()->precipitationtype,scenarioengine::PrecipitationType::SNOW);
    EXPECT_EQ(new_environment.GetPrecipitation()->precipitationintensity,environment.GetPrecipitation()->precipitationintensity);
    EXPECT_EQ(new_environment.GetPrecipitation()->precipitationtype,environment.GetPrecipitation()->precipitationtype);
    EXPECT_TRUE(new_environment.IsPrecipitation());
    EXPECT_TRUE(environment.IsPrecipitation());

    new_environment.SetSun(2,1,10000);
    environment.UpdateEnvironment(&new_environment);
    EXPECT_EQ(new_environment.GetSun()->azimuth,2);
    EXPECT_EQ(new_environment.GetSun()->elevation,1);
    EXPECT_EQ(new_environment.GetSun()->intensity,10000);
    EXPECT_EQ(new_environment.GetSun()->azimuth,environment.GetSun()->azimuth);
    EXPECT_EQ(new_environment.GetSun()->intensity,environment.GetSun()->intensity);
    EXPECT_EQ(new_environment.GetSun()->elevation,environment.GetSun()->elevation);
    EXPECT_TRUE(new_environment.IsSun());
    EXPECT_TRUE(environment.IsSun());
    
    new_environment.SetWind(2,10);
    environment.UpdateEnvironment(&new_environment);
    EXPECT_EQ(new_environment.GetWind()->direction,2);
    EXPECT_EQ(new_environment.GetWind()->speed,10);
    EXPECT_EQ(new_environment.GetWind()->direction,environment.GetWind()->direction);
    EXPECT_EQ(new_environment.GetWind()->speed,environment.GetWind()->speed);
    EXPECT_TRUE(new_environment.GetWind());
    EXPECT_TRUE(environment.GetWind());

    new_environment.SetRoadCondition(1);
    environment.UpdateEnvironment(&new_environment);
    EXPECT_EQ(new_environment.GetRoadCondition()->frictionscalefactor,1);
    EXPECT_EQ(new_environment.GetRoadCondition()->frictionscalefactor,environment.GetRoadCondition()->frictionscalefactor);
    EXPECT_TRUE(new_environment.IsRoadCondition());
    EXPECT_TRUE(environment.IsRoadCondition());

}

TEST(EnvironmentTest, Parsing)
{
    // Set up mock xml_node
    pugi::xml_document doc;
    pugi::xml_node actionNode = doc.append_child("Action");
    pugi::xml_node envActionNode = actionNode.append_child("EnvironmentAction");
    pugi::xml_node envNode = envActionNode.append_child("Environment");
    // Time of day
    pugi::xml_node todNode = envNode.append_child("TimeOfDay");
    todNode.append_attribute("animation").set_value(false);
    todNode.append_attribute("dateTime").set_value("2021-12-02T11:30:30");

    // Weather and attributes
    pugi::xml_node weatherNode = envNode.append_child("Weather");
    weatherNode.append_attribute("cloudState").set_value("cloudy");
    weatherNode.append_attribute("temperature").set_value("4");
    weatherNode.append_attribute("atmosphericPressure").set_value("1000");
    // Sun
    pugi::xml_node sunNode = weatherNode.append_child("Sun");
    sunNode.append_attribute("azimuth").set_value(0.5);
    sunNode.append_attribute("intensity").set_value(300.1);
    sunNode.append_attribute("elevation").set_value(3.2);
    // Fog
    pugi::xml_node fogNode = weatherNode.append_child("Fog");
    fogNode.append_attribute("visualRange").set_value(1000);
    pugi::xml_node fogBoundingboxNode = fogNode.append_child("BoundingBox");
    pugi::xml_node bbCenterNode = fogBoundingboxNode.append_child("Center");
    bbCenterNode.append_attribute("x").set_value(1);
    bbCenterNode.append_attribute("y").set_value(1);
    bbCenterNode.append_attribute("z").set_value(1.1);
    pugi::xml_node bbDimNode = fogBoundingboxNode.append_child("Dimensions");
    bbDimNode.append_attribute("width").set_value(1.2);
    bbDimNode.append_attribute("length").set_value(1.3);
    bbDimNode.append_attribute("height").set_value(1.4);
    // Precipitation
    pugi::xml_node precipNode = weatherNode.append_child("Precipitation");
    precipNode.append_attribute("precipitationType").set_value("dry");
    precipNode.append_attribute("precipitationIntensity").set_value(0.0);
    // Wind
    pugi::xml_node windNode = weatherNode.append_child("Wind");
    windNode.append_attribute("direction").set_value(1.1);
    windNode.append_attribute("speed").set_value(10);

    // Road condition
    pugi::xml_node roadCondNode = envNode.append_child("RoadCondition");
    roadCondNode.append_attribute("frictionScaleFactor").set_value(0.5);

    // Test using ScenarioReader
    Entities entities;
    Catalogs catalogs;
    OSCEnvironment environment;
    ScenarioReader reader(&entities, &catalogs, &environment);

    OSCGlobalAction *globalAct = reader.parseOSCGlobalAction(actionNode);
    EnvironmentAction *envAct = static_cast<EnvironmentAction*>(globalAct);
    OSCEnvironment oscEnv = envAct->new_environment_;

    EXPECT_EQ(oscEnv.GetTimeOfDay()->animation, todNode.attribute("animation").as_bool());
    EXPECT_EQ(oscEnv.GetTimeOfDay()->datetime, todNode.attribute("dateTime").value());

    EXPECT_EQ(oscEnv.GetCloudState(), scenarioengine::CloudState::CLOUDY);
    EXPECT_NEAR(oscEnv.GetTemperature(), weatherNode.attribute("temperature").as_double(), 1e-5);
    EXPECT_NEAR(oscEnv.GetAtmosphericPressure(), weatherNode.attribute("atmosphericPressure").as_double(), 1e-5);

    EXPECT_NEAR(oscEnv.GetSun()->azimuth, sunNode.attribute("azimuth").as_double(), 1e-5);
    EXPECT_NEAR(oscEnv.GetSun()->intensity, sunNode.attribute("intensity").as_double(), 1e-5);
    EXPECT_NEAR(oscEnv.GetSun()->elevation, sunNode.attribute("elevation").as_double(), 1e-5);

    EXPECT_NEAR(oscEnv.GetFog()->visibility_range, fogNode.attribute("visualRange").as_float(), 1e-5);
    EXPECT_NEAR(oscEnv.GetFog()->boundingbox.center_.x_, bbCenterNode.attribute("x").as_float(), 1e-5);
    EXPECT_NEAR(oscEnv.GetFog()->boundingbox.center_.y_, bbCenterNode.attribute("y").as_float(), 1e-5);
    EXPECT_NEAR(oscEnv.GetFog()->boundingbox.center_.z_, bbCenterNode.attribute("z").as_float(), 1e-5);
    EXPECT_NEAR(oscEnv.GetFog()->boundingbox.dimensions_.width_, bbDimNode.attribute("width").as_float(), 1e-5);
    EXPECT_NEAR(oscEnv.GetFog()->boundingbox.dimensions_.length_, bbDimNode.attribute("length").as_float(), 1e-5);
    EXPECT_NEAR(oscEnv.GetFog()->boundingbox.dimensions_.height_, bbDimNode.attribute("height").as_float(), 1e-5);

    EXPECT_EQ(oscEnv.GetPrecipitation()->precipitationtype, scenarioengine::PrecipitationType::DRY);
    EXPECT_NEAR(oscEnv.GetPrecipitation()->precipitationintensity, precipNode.attribute("precipitationIntensity").as_double(), 1e-5);

    EXPECT_NEAR(oscEnv.GetWind()->direction, windNode.attribute("direction").as_double(), 1e-5);
    EXPECT_NEAR(oscEnv.GetWind()->speed, windNode.attribute("speed").as_double(), 1e-5);

    EXPECT_NEAR(oscEnv.GetRoadCondition()->frictionscalefactor, roadCondNode.attribute("frictionScaleFactor").as_double(), 1e-5);
}

#if LOG_TO_CONSOLE
static void log_callback(const char* str)
{
    printf("%s\n", str);
}
#endif

int main(int argc, char **argv)
{
#if LOG_TO_CONSOLE
    if (!(Logger::Inst().IsCallbackSet()))
    {
        Logger::Inst().SetCallback(log_callback);
    }
#endif
    //testing::GTEST_FLAG(filter) = "suite*:test";

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}