#include <catch2/catch_test_macros.hpp>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>


TEST_CASE("Get wheels", "[DiffDrive]")
{
    turtlelib::DiffDrive robot;
    turtlelib::Wheels test_wheels = {2.24, 1.19};
    robot.set_wheels(test_wheels);

    turtlelib::Wheels actual_wheels = robot.get_wheels();

    REQUIRE_THAT(actual_wheels.phi_r, Catch::Matchers::WithinAbs(test_wheels.phi_r, 1e-5));
    REQUIRE_THAT(actual_wheels.phi_l, Catch::Matchers::WithinAbs(test_wheels.phi_l, 1e-5));

}

TEST_CASE("Forward Kinematics", "[DiffDrive]")
{
    SECTION("Robot drives forward")
    {
        // Initialize the DiffDrive robot
        // wheel_track, wheel_radius, wheels, q
        turtlelib::DiffDrive robot(0.16, 0.33, {0.0, 0.0}, {0.0, 0.0, 0.0});

        // Set the change in wheel position
        turtlelib::Wheels delta_wheels{turtlelib::PI/2, turtlelib::PI/2};

        // Update the robot config accordingly
        robot.forward_kinematic_update(delta_wheels);

        turtlelib::RobotConfig robot_config = robot.get_robot_config();
        // theta, x, y
        turtlelib::RobotConfig expected_config{0.0, 0.51836, 0.0};

        REQUIRE_THAT(robot_config.theta, Catch::Matchers::WithinAbs(expected_config.theta, 1e-5));
        REQUIRE_THAT(robot_config.x, Catch::Matchers::WithinAbs(expected_config.x, 1e-5));
        REQUIRE_THAT(robot_config.y, Catch::Matchers::WithinAbs(expected_config.y, 1e-5));

    }

    SECTION("Robot pure rotation (90 degrees CCW)")
    {
        // Initialize the DiffDrive robot
        // wheel_track, wheel_radius, wheels, q
        turtlelib::DiffDrive robot(2.0, 1.0, {0.0, 0.0}, {0.0, 0.0, 0.0});

        // Set the change in wheel position
        turtlelib::Wheels delta_wheels{-turtlelib::PI, turtlelib::PI};

        // Update the robot config accordingly
        robot.forward_kinematic_update(delta_wheels);

        turtlelib::RobotConfig robot_config = robot.get_robot_config();
        // theta, x, y
        turtlelib::RobotConfig expected_config{turtlelib::PI, 0.0, 0.0};

        REQUIRE_THAT(robot_config.theta, Catch::Matchers::WithinAbs(expected_config.theta, 1e-5));
        REQUIRE_THAT(robot_config.x, Catch::Matchers::WithinAbs(expected_config.x, 1e-5));
        REQUIRE_THAT(robot_config.y, Catch::Matchers::WithinAbs(expected_config.y, 1e-5));

    }

    SECTION("Robot follows arc")
    {
        // Initialize the DiffDrive robot
        // wheel_track, wheel_radius, wheels, q
        turtlelib::DiffDrive robot(2.0, 1.0, {0.0, 0.0}, {0.0, 0.0, 0.0});

        // Set the change in wheel position
        turtlelib::Wheels delta_wheels{1.5*turtlelib::PI, turtlelib::PI};

        // Update the robot config accordingly
        robot.forward_kinematic_update(delta_wheels);

        turtlelib::RobotConfig robot_config = robot.get_robot_config();
        // theta, x, y
        turtlelib::RobotConfig expected_config{-turtlelib::PI/4, 3.535533, -1.464466};

        REQUIRE_THAT(robot_config.theta, Catch::Matchers::WithinAbs(expected_config.theta, 1e-5));
        REQUIRE_THAT(robot_config.x, Catch::Matchers::WithinAbs(expected_config.x, 1e-5));
        REQUIRE_THAT(robot_config.y, Catch::Matchers::WithinAbs(expected_config.y, 1e-5));

    }
    
}

TEST_CASE("Inverse Kinematics", "[DiffDrive]")
{
    SECTION("Robot drives forward")
    {
        // Initialize the DiffDrive robot
        // wheel_track, wheel_radius, wheels, q
        turtlelib::DiffDrive robot(2.0, 1.0, {0.0, 0.0}, {0.0, 0.0, 0.0});

        // Initialize a desired twist (theta, x, y)
        turtlelib::Twist2D twist{0.0, 3.4, 0.0};

        // Do twist
        turtlelib::Wheels required_wheels = robot.inverse_kinematics(twist);

        turtlelib::Wheels expected_wheels{3.4, 3.4};

        REQUIRE_THAT(required_wheels.phi_l, Catch::Matchers::WithinAbs(required_wheels.phi_r, 1e-5));
        REQUIRE_THAT(required_wheels.phi_l, Catch::Matchers::WithinAbs(expected_wheels.phi_l, 1e-5));
        REQUIRE_THAT(required_wheels.phi_r, Catch::Matchers::WithinAbs(expected_wheels.phi_r, 1e-5));

    }

    SECTION("Robot pure rotation")
    {
        // Initialize the DiffDrive robot
        // wheel_track, wheel_radius, wheels, q
        turtlelib::DiffDrive robot(2.0, 1.0, {0.0, 0.0}, {0.0, 0.0, 0.0});

        // Initialize a desired twist (theta, x, y)
        turtlelib::Twist2D twist{turtlelib::PI/3, 0.0, 0.0};

        // Compute IK
        turtlelib::Wheels required_wheels = robot.inverse_kinematics(twist);

        turtlelib::Wheels expected_wheels{-1.0471975512, 1.0471975512};

        REQUIRE_THAT(required_wheels.phi_l, Catch::Matchers::WithinAbs(-required_wheels.phi_r, 1e-5));
        REQUIRE_THAT(required_wheels.phi_l, Catch::Matchers::WithinAbs(expected_wheels.phi_l, 1e-5));
        REQUIRE_THAT(required_wheels.phi_r, Catch::Matchers::WithinAbs(expected_wheels.phi_r, 1e-5));

    }

    SECTION("Robot follows arc")
    {
        // Initialize the DiffDrive robot
        // wheel_track, wheel_radius, wheels, q
        turtlelib::DiffDrive robot(2.0, 1.0, {0.0, 0.0}, {0.0, 0.0, 0.0});

        // Initialize a desired twist (theta, x, y)
        turtlelib::Twist2D twist{turtlelib::PI/3, 16.93, 0.0};

        // Compute IK
        turtlelib::Wheels required_wheels = robot.inverse_kinematics(twist);

        // Expected change in position in radians
        turtlelib::Wheels expected_wheels{15.8828024488, 17.9771975512};

        REQUIRE_THAT(required_wheels.phi_l, Catch::Matchers::WithinAbs(expected_wheels.phi_l, 1e-5));
        REQUIRE_THAT(required_wheels.phi_r, Catch::Matchers::WithinAbs(expected_wheels.phi_r, 1e-5));

    }

    SECTION("Impossible-to-follow twist")
    {
        // Initialize the DiffDrive robot
        // wheel_track, wheel_radius, wheels, q
        turtlelib::DiffDrive robot(2.0, 1.0, {0.0, 0.0}, {0.0, 0.0, 0.0});

        // Initialize a desired twist (theta, x, y)
        turtlelib::Twist2D twist{0.0, 3.4, 0.1};

        // ########## Begin_Citation [8] ##########

        REQUIRE_THROWS_AS(robot.inverse_kinematics(twist), std::logic_error);

        // ########## End_Citation [8] ##########
    }
}


