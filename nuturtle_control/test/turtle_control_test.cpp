// ########## Begin_Citation [14] ##########

#include <chrono>
#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
geometry_msgs::msg::Twist twist = geometry_msgs::msg::Twist();
sensor_msgs::msg::JointState joint_state;

double left_wheel_test;
double right_wheel_test;

void test_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
{
  left_wheel_test = msg->left_velocity;
  right_wheel_test = msg->right_velocity;
}

void senor_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  joint_state.name = msg->name;
  joint_state.position = msg->position;
  joint_state.velocity = msg->velocity;
}

geometry_msgs::msg::Twist twist1 = geometry_msgs::msg::Twist();

double left_wheel_test1;
double right_wheel_test1;

void test1_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
{
  left_wheel_test1 = msg->left_velocity;
  right_wheel_test1 = msg->right_velocity;
}

TEST_CASE("Pure Translation for wheel cmds", "[nuturtle_control]")
{
  auto node = rclcpp::Node::make_shared("turtle_control");
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  twist.linear.x = 0.1;
  twist.angular.z = 0.0;
  auto sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd", 10, &test_callback);

  rclcpp::Time start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < 1s))
  {
    rclcpp::spin_some(node);
    pub->publish(twist);
  }

  CHECK_THAT(left_wheel_test, Catch::Matchers::WithinAbs(63.0, 1e-5));
  CHECK_THAT(right_wheel_test, Catch::Matchers::WithinAbs(63.0, 1e-5));
};

TEST_CASE("Pure rotation", "[Rotation]")
{
  auto node = rclcpp::Node::make_shared("turtle_control");
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  twist1.linear.x = 0.0;
  twist1.angular.z = 0.2;
  auto sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd", 10, &test1_callback);

  rclcpp::Time start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < 1s))
  {
    rclcpp::spin_some(node);
    pub->publish(twist1);
  }

  CHECK_THAT(left_wheel_test1, Catch::Matchers::WithinAbs(-10.0, 1e-5));
  CHECK_THAT(right_wheel_test1, Catch::Matchers::WithinAbs(10.0, 1e-5));
};

nuturtlebot_msgs::msg::SensorData senor = nuturtlebot_msgs::msg::SensorData();
nuturtlebot_msgs::msg::SensorData senor1 = nuturtlebot_msgs::msg::SensorData();

// ########## End_Citation [14] ##########
