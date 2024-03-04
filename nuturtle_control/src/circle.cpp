/// \file
/// \brief Circle motion controller for the robot.
///
///
/// PARAMETERES:
///   \param freq (int): The frequency at which the control loop runs.
///
/// PUBLISHES:
///   \param cmd_vel (geometry_msgs::msg::Twist): Publishes the velocity commands for the robot to move in a circle.
///
/// SUBSCRIBES:
/// \param control (nuturtle_control::srv::Control): Services to change the velocity and radius of the circle motion.
/// \param reverse (std_srvs::srv::Empty): Service to reverse the direction of the circle motion.
/// \param stop (std_srvs::srv::Empty): Service to stop the circle motion.

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "std_srvs/srv/empty.hpp"

/// \brief  The Circle class provides an interface for moving the robot
///         along an arc.
class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {
    // Parameters
    declare_parameter("freq", 100);

    int freq_ = get_parameter("freq").get_value<int>();

    // Publishers
    cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Services
    control_init = create_service<nuturtle_control::srv::Control>(
      "/control",
      std::bind(
        &Circle::control_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    reverse_init = create_service<std_srvs::srv::Empty>(
      "/reverse",
      std::bind(
        &Circle::reverse_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    stop_init = create_service<std_srvs::srv::Empty>(
      "/stop",
      std::bind(
        &Circle::stop_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    // Timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / freq_),
      std::bind(&Circle::timer_callback, this));
  }

private:
  /// \brief Main timer callback.
  void timer_callback()
  {
    if (do_arc_ == true) {
      // If not stopped, publish the arc's twist
      cmd_vel_pub->publish(arc_);
    }

  }

  /// \brief Callback for the control service to adjust circle properties.
  /// \param request Service request containing the desired velocity and radius.
  void control_callback(
    std::shared_ptr<nuturtle_control::srv::Control::Request> request,
    std::shared_ptr<nuturtle_control::srv::Control::Response>)
  {
    // Changes the arc values to new values specified in service call
    arc_.linear.x = request->velocity; // changed
    arc_.angular.z = request->velocity / request->radius; // changed

    do_arc_ = true;
  }

  /// \brief Callback for the reverse service used to reverse the direction of circle motion.
  void reverse_callback(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    // Simply reverses the current direction
    arc_.linear.x = -arc_.linear.x;
    arc_.angular.z = -arc_.angular.z;

    do_arc_ = true;
  }

  /// \brief Callback for the stop service used to stop the circle motion.
  void stop_callback(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    // Stops all motion
    arc_.linear.x = 0.0;
    arc_.angular.z = 0.0;
    cmd_vel_pub->publish(arc_);

    // Stop publishing to cmd_vel
    do_arc_ = false;
  }

// Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

// Services
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_init;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_init;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_init;

// Timer
  rclcpp::TimerBase::SharedPtr timer_;

// Variables
  geometry_msgs::msg::Twist arc_;
  bool do_arc_ = true;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
