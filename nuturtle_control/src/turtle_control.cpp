/// \file turtle_control.cpp
/// \brief Node for controlling the turtlebot's wheel movements based on cmd_vel commands and sensor data.
///
/// PARAMETERS:
///   \param wheel_radius (double): The radius of the robot's wheels.
///   \param track_width (double): The distance between the centers of the two wheels.
///   \param motor_cmd_max (double): The maximum command value for the robot's motors.
///   \param motor_cmd_per_rad_sec (double): The command value per radian per second for the motors.
///   \param encoder_ticks_per_rad (double): The number of encoder ticks per radian of wheel rotation.
///
/// PUBLISHES:
///   \param wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): Commands for the left and right wheel velocities.
///   \param joint_states (sensor_msgs::msg::JointState): The joint states of the TurtleBot's wheels.
///
/// SUBSCRIBES:
///   \param cmd_vel (geometry_msgs::msg::Twist): The desired linear and angular velocities for the robot.
///   \param sensor_data (nuturtlebot_msgs::msg::SensorData): Encoder data from the robot's wheel sensors.


#include <cstdio>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"


/// \brief Node for controlling the turtlebot's wheel movement based on joint states.
class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    // Parameters
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("motor_cmd_max", -1.0);
    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    declare_parameter("encoder_ticks_per_rad", -1.0);

    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();
    motor_cmd_max_ = get_parameter("motor_cmd_max").get_parameter_value().get<double>();
    motor_cmd_per_rad_sec_ =
      get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    encoder_ticks_per_rad_ =
      get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();

    // Publishers
    wheel_cmd_pub = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("/wheel_cmd", 10);
    joint_states_pub = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Subscribers
    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1));
    sensor_data_sub = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(
        &TurtleControl::sensor_data_callback, this,
        std::placeholders::_1));

    // Setup Functions
    check_params();

    robot = turtlelib::DiffDrive(track_width_, wheel_radius_, {0.0, 0.0}, {0.0, 0.0, 0.0});
  }

private:
  /// \brief Callback for cmd_vel messages.
  /// \param msg The cmd_vel message containing the desired velocities.
  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {
    // Construct a Twist2D using received cmd_vel message
    turtlelib::Twist2D twist;
    twist.omega = msg.angular.z;
    twist.x = msg.linear.x;
    twist.y = msg.linear.y;

    // Perform inverse kinematics to get the wheel commands (Wheels)
    wheels = robot.inverse_kinematics(twist);

    wheels.phi_l = static_cast<int>(wheels.phi_l / motor_cmd_per_rad_sec_); // and multiply by rate?
    wheels.phi_r = static_cast<int>(wheels.phi_r / motor_cmd_per_rad_sec_);

    wheel_cmd_.left_velocity = wheels.phi_l;
    wheel_cmd_.right_velocity = wheels.phi_r;

    // Ensure motor (wheel) commands are within specified interval
    if (wheel_cmd_.left_velocity > motor_cmd_max_) {
      wheel_cmd_.left_velocity = motor_cmd_max_;
    } else if (wheel_cmd_.left_velocity < -motor_cmd_max_) {
      wheel_cmd_.left_velocity = -motor_cmd_max_;
    }
    if (wheel_cmd_.right_velocity > motor_cmd_max_) {
      wheel_cmd_.right_velocity = motor_cmd_max_;
    } else if (wheel_cmd_.right_velocity < -motor_cmd_max_) {
      wheel_cmd_.right_velocity = -motor_cmd_max_;
    }

    wheel_cmd_pub->publish(wheel_cmd_);

  }

  /// \brief Callback for sensor_data messages.
  /// \param msg The sensor_data message containing the encoder data and publishes joint states.
  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {

    // Create a JointState message
    sensor_msgs::msg::JointState js;

    // Create the message (header, name, position, velocity, effort)
    js.header.stamp = msg.stamp;
    js.name = {"wheel_left_joint", "wheel_right_joint"};

    // Load position and velocity into joint state message
    if (flag_stamp == -1.0) { // If it's the first reading...
      js.position = {0.0, 0.0};
      js.velocity = {0.0, 0.0};
    } else {
      // Time since last reading
      double elapsed_time = msg.stamp.sec + msg.stamp.nanosec * 1e-9 - flag_stamp; // Convert units to seconds

      js.position =
      {msg.left_encoder / encoder_ticks_per_rad_,
        msg.right_encoder / encoder_ticks_per_rad_};

      if (elapsed_time > 0.0) {
        js.velocity = {js.position.at(0) / elapsed_time,
          js.position.at(1) / elapsed_time};
      }
      
    }

    // Set stamp and publish joint states
    flag_stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9; // Can also use: get_clock()->now()

    // RCLCPP_ERROR_STREAM(
    //   this->get_logger(), "left: " << js.position.at(0) << " right: " << js.position.at(1));

    
    joint_states_pub->publish(js);

  }

  /// \brief Checks if all required parameters are defined, and logs an error if any are missing.
  void check_params()
  {
    if (wheel_radius_ < 0.0 ||
      track_width_ < 0.0 ||
      motor_cmd_max_ < 0.0 ||
      motor_cmd_per_rad_sec_ < 0.0 ||
      encoder_ticks_per_rad_ < 0.0)
    {
      RCLCPP_ERROR_STREAM(
        this->get_logger(), "Not all required parameters are defined in diff_params.yaml.");
      rclcpp::shutdown();
    }
  }

// Publishers
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;

// Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub;

// Variables
  int rate_;
  double wheel_radius_;
  double track_width_;
  double motor_cmd_max_;
  double motor_cmd_per_rad_sec_;
  double encoder_ticks_per_rad_;
  double flag_stamp = -1.0;
  turtlelib::DiffDrive robot;
  // turtlelib::Twist2D twist;
  turtlelib::Wheels wheels;
  nuturtlebot_msgs::msg::WheelCommands wheel_cmd_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
