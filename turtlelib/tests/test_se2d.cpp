#include <catch2/catch_test_macros.hpp>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>

TEST_CASE("Prints the Twist2D in the format [w x y]", "[Twist2D]")
{
    turtlelib::Twist2D tw{0.5, 4.4, 3.2}; // Initialize a turtlelib Twist2D object

    std::ostringstream output; // Create an output stream to capture the printed output

    output << tw;

    REQUIRE(output.str() == "[0.5 4.4 3.2]");
}

TEST_CASE("Reads the Twist2D in the format [w x y] or w x y", "[Twist2D]") // Henry, Buron
{
    // Create a Twist2D object to store the parsed values
    turtlelib::Twist2D tw;
    SECTION("Read twist in format [w x y]")
    {
        // Create a string containing the input data in the format [w x y]
        std::string input_string = "[1.4 3.8 6.4]";

        // Create an input string stream and initialize it with the input string
        std::istringstream input_sstream(input_string);

        // Attempt to read the Twist2D object from the input string stream
        input_sstream >> tw;

        // Check if the parsed values match the expected values (with a tolerance)
        REQUIRE_THAT(tw.omega, Catch::Matchers::WithinAbs(1.4, 1e-5));
        REQUIRE_THAT(tw.x, Catch::Matchers::WithinAbs(3.8, 1e-5));
        REQUIRE_THAT(tw.y, Catch::Matchers::WithinAbs(6.4, 1e-5));
    }
    SECTION("Read twist in format w x y")
    {
        std::string input_string = "[1.2 1.8 8.4]";
        std::istringstream input_sstream(input_string);

        input_sstream >> tw;

        REQUIRE_THAT(tw.omega, Catch::Matchers::WithinAbs(1.2, 1e-5));
        REQUIRE_THAT(tw.x, Catch::Matchers::WithinAbs(1.8, 1e-5));
        REQUIRE_THAT(tw.y, Catch::Matchers::WithinAbs(8.4, 1e-5));
    }
}

TEST_CASE("Identity Transformation Test", "[Transform2D]") // Henry, Buron
{
    turtlelib::Transform2D identity;

    SECTION("Testing transform")
    {
        turtlelib::Vector2D vec{3.5, 4.1};
        turtlelib::Vector2D transformed_vector = identity(vec);
        REQUIRE_THAT(transformed_vector.x, Catch::Matchers::WithinAbs(3.5, 1e-5));
        REQUIRE_THAT(transformed_vector.y, Catch::Matchers::WithinAbs(4.1, 1e-5));
    }
}

TEST_CASE("Translation", "[Transform]") // Henry, Buron
{
    turtlelib::Vector2D vector{9.9, 8.8};
    turtlelib::Transform2D test_translation(vector);

    // Check that the transform was a pure translation
    REQUIRE_THAT(test_translation.translation().x, Catch::Matchers::WithinAbs(9.9, 1e-5));
    REQUIRE_THAT(test_translation.translation().y, Catch::Matchers::WithinAbs(8.8, 1e-5));
    REQUIRE_THAT(test_translation.rotation(), Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("Rotation", "[Transform]") // Henry, Buron
{
    double radians{3.14};
    turtlelib::Transform2D test_rotation(radians);

    // Check that the transform was a pure rotation
    REQUIRE_THAT(test_rotation.translation().x, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(test_rotation.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(test_rotation.rotation(), Catch::Matchers::WithinAbs(3.14, 1e-5));
}

TEST_CASE("Translation and rotation", "[Transform]") // Henry, Buron
{
    turtlelib::Vector2D vector{1.2, 3.4};
    double radians{5.6};
    turtlelib::Transform2D test_transform(vector, radians);

    REQUIRE_THAT(test_transform.translation().x, Catch::Matchers::WithinAbs(1.2, 1e-5));
    REQUIRE_THAT(test_transform.translation().y, Catch::Matchers::WithinAbs(3.4, 1e-5));
    REQUIRE_THAT(test_transform.rotation(), Catch::Matchers::WithinAbs(5.6, 1e-5));
}

TEST_CASE("Applying a transformation (operator()) to a 2D point.", "[Transform]") // Henry, Buron
{
    // Define the original point
    turtlelib::Point2D original_point{2.0, 4.5};

    // Define the transformation
    turtlelib::Transform2D transform_test(turtlelib::Vector2D{1.0, -2.6}, -1.57079632);

    // Apply the transformation
    turtlelib::Point2D transformed_point = transform_test(original_point);

    // Define the expected result point
    turtlelib::Point2D expected_point{5.5, -4.6};

    REQUIRE_THAT(transformed_point.x, Catch::Matchers::WithinAbs(expected_point.x, 1e-5));
    REQUIRE_THAT(transformed_point.y, Catch::Matchers::WithinAbs(expected_point.y, 1e-5));
}

TEST_CASE("Applying a transformation (operator()) to a 2D vector", "[Transform]") // Henry, Buron
{
    // Define the original vector to be transformed
    turtlelib::Vector2D original_vector{1.0, 1.0};

    // Define the transformation
    turtlelib::Transform2D transform_test(turtlelib::Vector2D{3.0, 4.0}, 1.57079632);

    // Apply the transformation to the vector
    turtlelib::Vector2D transformed_vector = transform_test(original_vector);

    // Define the expected result vector
    turtlelib::Vector2D expected_vector{2.0, 5.0};

    REQUIRE_THAT(transformed_vector.x, Catch::Matchers::WithinAbs(expected_vector.x, 1e-3));
    REQUIRE_THAT(transformed_vector.y, Catch::Matchers::WithinAbs(expected_vector.y, 1e-3));
}

TEST_CASE("Applying a transformation (operator()) to a 2D twist", "[Transform]") // Henry, Buron
{
    // Define the transform values from A to B
    turtlelib::Transform2D T_ab{turtlelib::Vector2D{1.2, 3.4}, 0.4};

    // Define the Twist of frame B
    turtlelib::Twist2D T_b{turtlelib::PI / 3, 5.6, 7.8};

    // Apply the transformation A --> B
    turtlelib::Twist2D T_a = T_ab(T_b);

    // Define the expected b frame
    turtlelib::Twist2D T_a_expected{1.047197, 5.68095, 8.108381};

    REQUIRE_THAT(T_a.omega, Catch::Matchers::WithinAbs(T_a_expected.omega, 1e-5));
    REQUIRE_THAT(T_a.x, Catch::Matchers::WithinAbs(T_a_expected.x, 1e-5));
    REQUIRE_THAT(T_a.y, Catch::Matchers::WithinAbs(T_a_expected.y, 1e-5));
}

TEST_CASE("Inverse transformation", "[Transform]") // Henry, Buron
{
    // Define a transform object. It does a transform.
    turtlelib::Transform2D original_transform{turtlelib::Vector2D{2.0, 9.0}, turtlelib::PI / 3};

    // Apply the inverse transformation
    turtlelib::Transform2D inverse_transformation = original_transform.inv();

    // Define the expected transform
    turtlelib::Transform2D expected_transform{turtlelib::Vector2D{-1.0, -7.0}, -turtlelib::PI / 2};

    REQUIRE_THAT(inverse_transformation.rotation(), Catch::Matchers::WithinAbs(-turtlelib::PI / 3, 1e-5));
    REQUIRE_THAT(inverse_transformation.translation().x, Catch::Matchers::WithinAbs(-8.794228, 1e-5));
    REQUIRE_THAT(inverse_transformation.translation().y, Catch::Matchers::WithinAbs(-2.767949, 1e-5));
}

TEST_CASE("operator *=", "[Transform]") // Henry, Buron
{
    turtlelib::Vector2D tf_ab = {0.5, 2.5};
    double rot_ab = 2.0;
    turtlelib::Transform2D tf_ab1 = {tf_ab, rot_ab}, tf_ab2 = tf_ab1, tf_ab3 = tf_ab1;

    turtlelib::Vector2D T_bc = {4.0, 1.2};
    double rot_bc = turtlelib::PI / 2.0;
    turtlelib::Transform2D tf_bc = {T_bc, rot_bc};

    REQUIRE_THAT((tf_ab3 *= tf_bc).rotation(), Catch::Matchers::WithinAbs(turtlelib::PI / 2.0 + 2.0, 1e-5));
    REQUIRE_THAT((tf_ab1 *= tf_bc).translation().x, Catch::Matchers::WithinAbs(-2.255744, 1e-5));
    REQUIRE_THAT((tf_ab2 *= tf_bc).translation().y, Catch::Matchers::WithinAbs(5.63781, 1e-5));
}

TEST_CASE("Human-readable transform", "[Transform]") // Henry, Buron
{
    // Ensure that stream insertion operator gives correct format...

    // Create string stream object
    std::stringstream sstream;

    // Build Transform2D object
    turtlelib::Transform2D transform{turtlelib::Vector2D{4.0, 2.1}, turtlelib::PI / 4};

    // Define expected string
    std::string expected_string = "deg: 45 x: 4 y: 2.1";

    // Put transform into string stream
    sstream << transform;

    // Check that the output matches the expected value
    REQUIRE(sstream.str() == expected_string);
}

TEST_CASE("Read transformation with stream extraction operator >>", "[Transform]") // Henry, Buron
{
    turtlelib::Transform2D tf = turtlelib::Transform2D();
    std::stringstream sstream;
    sstream << "deg: 34 x: 1785 y: 0.01";
    sstream >> tf;
    REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(turtlelib::deg2rad(34), 1e-5));
    REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(1785.0, 1e-5));
    REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.01, 1e-5));
}

// Don't have a test case for final: operator*

TEST_CASE("Integrate a twist", "[Transform2D]")
{
    SECTION("Pure translation")
    {
        turtlelib::Twist2D twist{0.0, 3.0, 4.0};
        turtlelib::Transform2D translated = turtlelib::integrate_twist(twist);

        REQUIRE_THAT(translated.translation().x, Catch::Matchers::WithinAbs(3.0, 1e-5));
        REQUIRE_THAT(translated.translation().y, Catch::Matchers::WithinAbs(4.0, 1e-5));
    }

    SECTION("Pure rotation", "[Transform2D]")
    {
        turtlelib::Twist2D twist{turtlelib::PI, 0.0, 0.0};
        turtlelib::Transform2D rotated = turtlelib::integrate_twist(twist);

        REQUIRE_THAT(rotated.rotation(), Catch::Matchers::WithinAbs(3.141592, 1e-5));
    }

    SECTION("Translation and Rotation", "[Transform2D]")
    {
        turtlelib::Twist2D twist{turtlelib::PI/3, 6.66, -1.134427};
        turtlelib::Transform2D transformed = turtlelib::integrate_twist(twist);

        REQUIRE_THAT(transformed.rotation(), Catch::Matchers::WithinAbs(1.0471975512, 1e-5));
        REQUIRE_THAT(transformed.translation().x, Catch::Matchers::WithinAbs(6.0494246591, 1e-5));
        REQUIRE_THAT(transformed.translation().y, Catch::Matchers::WithinAbs(2.2417521857, 1e-5));
    }
}
