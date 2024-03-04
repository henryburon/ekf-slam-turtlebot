/// \file nusim.cpp
/// \brief Provides a simulated robot environment in RViz2.
///        Displays walls, obstacles, and turtlebot.
///
/// PARAMETERS:
///     \param rate (int): Timer frequency (Hz)
///     \param x0 (double): Initial x coordinate of the robot (m)
///     \param y0 (double): Initial y coordinate of the robot (m)
///     \param theta0 (double): Initial theta angle of the robot (radians)
///     \param walls.arena_x_length (double): Length of arena in world x direction (m)
///     \param walls.arena_y_length (double): Length of arena in world y direction (m)
///     \param obstacles.x (std::vector<double>): List of the obstacles' x coordinates (m)
///     \param obstacles.y (std::vector<double>): List of the obstacles' y coordinates (m)
///     \param obstacles.r (double): Radius of cylindrical obstacles (m)
///     \param motor_cmd_per_rad_sec (double): Motor command per radian per second
///     \param encoder_ticks_per_rad (double): Encoder ticks per radian
///     \param input_noise (double): Noise to add to the wheel velocities
///     \param slip_fraction (double): Fraction of slip to add to the wheel positions
///     \param basic_sensor_variance (double): Variance of the basic sensor
///     \param max_range (double): Maximum range of the sensor
///     \param collision_radius (double): Radius of the robot

/// PUBLISHES:
///     \param ~/timestep (std_msgs::msg::UInt64): current timestep of the simulation
///     \param ~/walls (visualization_msgs::msg::MarkerArray): MarkerArray of Walls in RViz2
///     \param ~/obstacles (visualization_msgs::msg::MarkerArray): MarkerArray of cylindrical obstacles in RViz2
///     \param /red/sensor_data (nuturtlebot_msgs::msg::SensorData): Sensor data from the red robot
///     \param /red/path (nav_msgs::msg::Path): Path of the red robot
///     \param /fake_sensor (visualization_msgs::msg::MarkerArray): Measured obstacles as a marker array
/// SUBSCRIBES:
///     \param /red/wheel_cmd: wheel commands for the red robot
/// SERVERS:
///      \param ~/reset (std_srvs::srv::Empty): Resets the simulation
///      \param ~/teleport (nusim::srv::Teleport): Teleports robot to a desired pose
/// CLIENTS:
///     None

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


using namespace std::chrono_literals;

/// \brief The Nusim class provides a simulated environment for the robot
///        It publishes the current timestep of the simulation, along
///        with an arena and cylindrical obstacles as markers in RViz2.
///        It offers services like reset and teleport.
///
///  \param rate (int): Timer frequency (Hz)
///  \param x0 (double): Initial x-coordinate of the robot (m)
///  \param y0 (double): Initial y-coordinate of the robot (m)
///  \param theta0 (double): Initial theta angle of the robot (radians)
///  \param x (double): Current x-coordinate of the robot (m)
///  \param y (double): Current y-coordinate of the robot (m)
///  \param theta (double): Current theta angle of the robot (radians)
///  \param walls.arena_x_length (double): Length of arena in world x direction (m)
///  \param walls.arena_y_length (double): Length of arena in world y direction (m)
///  \param obstacles.x (std::vector<double>): List of the obstacles' x-coordinates (m)
///  \param obstacles.y (std::vector<double>): List of the obstacles' y-coordinates (m)
///  \param obstacles.r (double): Radius of cylindrical obstacles (m)
/// \param motor_cmd_per_rad_sec (double): Motor command per radian per second

class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim"), timestep_(0)
  {
    // Parameters
    declare_parameter("rate", 200);
    declare_parameter("x0", -0.5);
    declare_parameter("y0", 0.7);
    declare_parameter("theta0", 1.28);
    declare_parameter("walls.arena_x_length", 6.0);
    declare_parameter("walls.arena_y_length", 6.0);
    declare_parameter("obstacles.x", std::vector<double>{});
    declare_parameter("obstacles.y", std::vector<double>{});
    declare_parameter("obstacles.r", 0.038);
    declare_parameter("motor_cmd_per_rad_sec", 0.024);
    declare_parameter("encoder_ticks_per_rad", 651.8986469);
    declare_parameter("input_noise", 0.0); // on scale of 0.001
    declare_parameter("slip_fraction", 0.0); // on scale of 0.0001
    declare_parameter("basic_sensor_variance", 0.00001);
    declare_parameter("max_range", 2.5);
    declare_parameter("collision_radius", 0.11);
    declare_parameter("angle_min", 0.0);
    declare_parameter("angle_max", 2 * 3.14159265359);
    declare_parameter("angle_increment", 3.14159265359 / 180.0);
    declare_parameter("time_increment", 0.000557413);
    declare_parameter("scan_time", 0.2066);
    declare_parameter("range_min", 0.119999);
    declare_parameter("range_max", 1.5);
    declare_parameter("num_samples", 360);
    declare_parameter("lidar_noise", 0.00001);

    rate_ = get_parameter("rate").get_parameter_value().get<int>();
    x0_ = get_parameter("x0").get_parameter_value().get<double>();
    y0_ = get_parameter("y0").get_parameter_value().get<double>();
    theta0_ = get_parameter("theta0").get_parameter_value().get<double>();
    arena_x_length_ = get_parameter("walls.arena_x_length").get_parameter_value().get<double>();
    arena_y_length_ = get_parameter("walls.arena_y_length").get_parameter_value().get<double>();
    obstacles_x_ = get_parameter("obstacles.x").get_parameter_value().get<std::vector<double>>();
    obstacles_y_ = get_parameter("obstacles.y").get_parameter_value().get<std::vector<double>>();
    obstacles_r_ = get_parameter("obstacles.r").get_parameter_value().get<double>();
    motor_cmd_per_rad_sec_ =
      get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    encoder_ticks_per_rad_ =
      get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();
    input_noise_ = get_parameter("input_noise").get_parameter_value().get<double>();
    slip_fraction_ = get_parameter("slip_fraction").get_parameter_value().get<double>();
    basic_sensor_variance_ = get_parameter("basic_sensor_variance").get_parameter_value().get<double>();
    max_range_ = get_parameter("max_range").get_parameter_value().get<double>();
    collision_radius_ = get_parameter("collision_radius").get_parameter_value().get<double>();
    angle_min_ = get_parameter("angle_min").get_parameter_value().get<double>();
    angle_max_ = get_parameter("angle_max").get_parameter_value().get<double>();
    angle_increment_ = get_parameter("angle_increment").get_parameter_value().get<double>();
    time_increment_ = get_parameter("time_increment").get_parameter_value().get<double>();
    scan_time_ = get_parameter("scan_time").get_parameter_value().get<double>();
    range_min_ = get_parameter("range_min").get_parameter_value().get<double>();
    range_max_ = get_parameter("range_max").get_parameter_value().get<double>();
    num_samples_ = get_parameter("num_samples").get_parameter_value().get<int>();
    lidar_noise_ = get_parameter("lidar_noise").get_parameter_value().get<double>();

    // Publishers
    timestep_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    walls_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);

    obstacles_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles", 10);

    red_sensor_data_pub =
      create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("red/path", 10);

    fake_sensor_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("fake_sensor", 10);

    laser_scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("red/laser_scan", 10);

    // Subscribers
    red_wheel_sub = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(&Nusim::red_wheel_callback, this, std::placeholders::_1));

    // Services
    reset_service = this->create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    teleport_service = this->create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Broadcasters
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Timers
    timer_ =
      this->create_wall_timer(
      std::chrono::milliseconds(10000 / rate_), // added an extra 0 to the numerator to account for the unknown 10x speedup
      std::bind(&Nusim::timer_callback, this));

    sensor_timer_ =
      this->create_wall_timer(0.2s, std::bind(&Nusim::sensor_timer_callback, this));

    lidar_timer_ =
      this->create_wall_timer(0.2s, std::bind(&Nusim::lidar_timer_callback, this));

    // Initialize robot position
    x_ = x0_;
    y_ = y0_;
    theta_ = theta0_;

    wall_thickness_ = 0.1;

    create_walls_();
    create_obstacles();
  }

private:

  /// \brief Callback function for lidar data (5 Hz)
  void lidar_timer_callback()
  {
    // at 5 Hz, publish a laser scan on the red/laser_scan topic
    get_lidar();
  }

  /// \brief Callback function for sensor data (5 Hz)
  void sensor_timer_callback()
  {
    // at 5 Hz, publish a marker array on the fake_sensor topic
    measure_obstacles();

  }

  /// \brief Main timer callback function
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_++;

    // Publish the UInt64 message
    timestep_publisher_->publish(message);

    // Read message content and assign it to corresponding tf variables
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";

    // Turtle only exists in 2D, so we set z coordinate to 0
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    // Likewise, turtle can only rotate around one axis -- z
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transform
    tf_broadcaster_->sendTransform(t);

    walls_publisher_->publish(wall_markers_array_);
    obstacles_publisher_->publish(obstacles_markers_array_);

    update_robot_position();
    update_wheel_positions();

    // Update path
    path_msg.header.stamp = get_clock()->now();
    path_msg.header.frame_id = "nusim/world";

    geometry_msgs::msg::PoseStamped pose_stamp_;
    pose_stamp_.pose.position.x = x_;
    pose_stamp_.pose.position.y = y_;
    pose_stamp_.pose.position.z = 0.0;
    pose_stamp_.pose.orientation.x = q.x();
    pose_stamp_.pose.orientation.y = q.y();
    pose_stamp_.pose.orientation.z = q.z();
    pose_stamp_.pose.orientation.w = q.w();
    path_msg.poses.push_back(pose_stamp_);

    // Publish path
    path_publisher_->publish(path_msg);

  }

  /// \brief Callback function for wheel commands from the red robot.
  /// \param msg The incoming wheel command message containing the commanded velocity for each wheel.
  void red_wheel_callback(const nuturtlebot_msgs::msg::WheelCommands & msg)
  {
    // Left and right wheel velocity, in "motor command units" (mcu)
    // For the turtlebot, each motor can be command with an integer velocity of between
    // -265 mcu and 265 mcu, and 1 mcu = 0.024 rad/sec

    wheel_vel_.phi_l = static_cast<double>(msg.left_velocity * motor_cmd_per_rad_sec_);
    wheel_vel_.phi_r = static_cast<double>(msg.right_velocity * motor_cmd_per_rad_sec_);

    // Add error/noise to the wheel velocities if either wheel_cmd is non-zero (i.e. it's not stopping)
    if (wheel_vel_.phi_l != 0.0 || wheel_vel_.phi_r != 0.0) {
      std::random_device rd;
      std::mt19937 gen(rd());

      // Mean of 0.0 and a variance of input_noise_
      std::normal_distribution<double> dist(0.0, std::sqrt(input_noise_));
      
      // Add noise to the wheel velocities
      wheel_vel_.phi_l += dist(gen);
      wheel_vel_.phi_r += dist(gen);
    }

  }

  /// \brief Updates the wheel positions based on the current wheel velocities and publishes the sensor data.
  /// \param msg The incoming wheel command message containing the velocities for the left and right wheels.
  void update_wheel_positions()
  {
    double unit_per_run = 1.0 / rate_;

    // Find the updated wheel position
    updated_wheel_pos_.phi_l = prev_wheel_pos_.phi_l + (wheel_vel_.phi_l * unit_per_run);
    updated_wheel_pos_.phi_r = prev_wheel_pos_.phi_r + (wheel_vel_.phi_r * unit_per_run);

    // Format as sensor data (in ticks)
    sensor_data_msg_.left_encoder = updated_wheel_pos_.phi_l * encoder_ticks_per_rad_;
    sensor_data_msg_.right_encoder = updated_wheel_pos_.phi_r * encoder_ticks_per_rad_;
    sensor_data_msg_.stamp = get_clock()->now();

    // Publish on red/sensor_data
    red_sensor_data_pub->publish(sensor_data_msg_);
  }

  /// \brief Updates the robot's position based on the change in wheel position.
  void update_robot_position()
  {
    // Do forward kinematics
    turtlelib::Wheels delta_wheels;
    delta_wheels.phi_l = updated_wheel_pos_.phi_l - prev_wheel_pos_.phi_l;
    delta_wheels.phi_r = updated_wheel_pos_.phi_r - prev_wheel_pos_.phi_r;

    robot_.forward_kinematic_update(delta_wheels);

    // Extract and set new positions
    x_ = robot_.get_robot_config().x;
    y_ = robot_.get_robot_config().y;
    theta_ = robot_.get_robot_config().theta;

    prev_wheel_pos_.phi_l = updated_wheel_pos_.phi_l;
    prev_wheel_pos_.phi_r = updated_wheel_pos_.phi_r;

    // Add slip to the wheels' position
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> err(-1.0 * slip_fraction_, slip_fraction_);

    prev_wheel_pos_.phi_l *= (1.0 + err(gen));
    prev_wheel_pos_.phi_r *= (1.0 + err(gen));

    // Detect collision
    detect_collision();

  }

  /// \brief Detects collision between the robot and the obstacles
  void detect_collision()
{
    x_detect = robot_.get_robot_config().x;
    y_detect = robot_.get_robot_config().y;
    theta_detect = robot_.get_robot_config().theta;

    // check if the robot is colliding with any of the (actual) obstacles
    for (size_t i = 0; i < obstacles_x_.size(); ++i) {
        double distance = measure_distance(x_detect, y_detect, obstacles_x_.at(i), obstacles_y_.at(i));
        if (distance < collision_radius_) {

            // calculate the distance to move
            double move_distance = (collision_radius_ + obstacles_r_) - distance; // equal to the intersection of the circles

            // get the direction to move
            turtlelib::Vector2D dir_vec = turtlelib::Vector2D{x_detect, y_detect} - turtlelib::Vector2D{obstacles_x_.at(i), obstacles_y_.at(i)}; // vector from obstacle to robot

            // normalize
            turtlelib::Vector2D dir_vec_norm = turtlelib::normalize_vector(dir_vec);

            // move but maintain the robot's orientation
            turtlelib::Vector2D new_pos = turtlelib::Vector2D{x_detect, y_detect} + dir_vec_norm * move_distance;
            robot_.set_robot_config({theta_detect, new_pos.x, new_pos.y});
        }
    }
}       

  /// \brief Measures the distance between two points
  double measure_distance(double x1, double y1, double x2, double y2)
  {
      double dx = x2 - x1;
      double dy = y2 - y1;
      return std::sqrt(dx * dx + dy * dy);
  }

  /// \brief Publishes the measured obstacles as a marker array
  void measure_obstacles()
  {
    // measure the position of each marker (i.e. add gaussian noise to the coordinates)
    std::vector<double> measured_x;
    std::vector<double> measured_y;
    for (size_t i = 0; i < obstacles_x_.size(); ++i) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> dist(0.0, std::sqrt(basic_sensor_variance_));
        measured_x.push_back(obstacles_x_.at(i) + dist(gen));
        measured_y.push_back(obstacles_y_.at(i) + dist(gen));
    }

    // find the distance from the robot to each marker, and detect if it's within the max range
    std::vector<double> distances;
    std::vector<bool> detected;
    for (size_t i = 0; i < obstacles_x_.size(); ++i) {
        double distance = measure_distance(x_, y_, measured_x.at(i), measured_y.at(i));
        distances.push_back(distance);
        detected.push_back(distance < max_range_);
    }

    // create the marker array
    visualization_msgs::msg::MarkerArray measured_obstacle_markers;

    for (size_t i = 0; i < obstacles_x_.size(); ++i) {
        visualization_msgs::msg::Marker fake_sensor_marker;
        fake_sensor_marker.header.frame_id = "nusim/world";
        fake_sensor_marker.header.stamp = get_clock()->now();
        fake_sensor_marker.id = i;
        fake_sensor_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        fake_sensor_marker.pose.position.x = measured_x.at(i);
        fake_sensor_marker.pose.position.y = measured_y.at(i);
        fake_sensor_marker.pose.position.z = 0.125;
        fake_sensor_marker.scale.x = 2.0 * obstacles_r_ * 1.05;
        fake_sensor_marker.scale.y = 2.0 * obstacles_r_ * 1.05;
        fake_sensor_marker.scale.z = 0.25;
        fake_sensor_marker.color.r = 1.0;
        fake_sensor_marker.color.g = 1.0;
        fake_sensor_marker.color.b = 0.0;
        fake_sensor_marker.color.a = 1.0;

        if (detected.at(i)) {
            fake_sensor_marker.action = visualization_msgs::msg::Marker::ADD;
        }
        else {
            fake_sensor_marker.action = visualization_msgs::msg::Marker::DELETE;
        }
        measured_obstacle_markers.markers.push_back(fake_sensor_marker);
    }

    fake_sensor_pub->publish(measured_obstacle_markers);
  }

// ########## Begin_Citation [15] ##########

/// \brief Creates a laser scan
  void get_lidar()
  {
    // Initialize a new LaserScan message
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = get_clock()->now();
    scan.header.frame_id = "red/base_scan";
    scan.angle_min = angle_min_;
    scan.angle_max = angle_max_;
    scan.angle_increment = angle_increment_;
    scan.time_increment =  time_increment_;
    scan.scan_time = scan_time_;
    scan.range_min = range_min_;
    scan.range_max = range_max_;

    scan.ranges.resize(num_samples_);

    // Loop over all samples
    for (int laser_sample = 0; laser_sample < num_samples_; laser_sample++) 
    {
      // Compute current angle of lidar (in world frame)
      double current_angle = angle_min_ + laser_sample * angle_increment_ + theta_;
      double min_distance = range_max_;

      double x_pos = std::cos(current_angle);
      double y_pos = std::sin(current_angle);

      std::vector<double> wall_distance;

      // Calculate distances to the u/d walls
      if (y_pos != 0) 
        {
          double upper_bound = (arena_x_length_ / 2.0 - y_) / y_pos;
          double lower_bound = (-arena_y_length_ / 2.0 - y_) / y_pos;

          // Add valid distances to the vector
          if (upper_bound > 0) wall_distance.push_back(upper_bound);
          if (lower_bound > 0) wall_distance.push_back(lower_bound);
        }

      // calculate distances to the l/r walls
      if (x_pos != 0) 
        {
          double right_bound = (arena_x_length_ / 2.0 - x_) / x_pos;
          double left_bound = (-arena_x_length_ / 2.0 - x_) / x_pos;

          if (right_bound > 0) wall_distance.push_back(right_bound);
          if (left_bound > 0) wall_distance.push_back(left_bound);
        }

      // find min distance to a wall
      if (!wall_distance.empty()) 
        {
          double min_wall_dist = *std::min_element(wall_distance.begin(), wall_distance.end());
          min_distance = std::min(min_distance, min_wall_dist);
        }

    // Check for intersection with obstacles
    for (size_t obs = 0; obs < obstacles_x_.size(); obs++) 
      {
        // Define start and end of the laser beam
        turtlelib::Point2D p1 = {x_, y_};
        turtlelib::Point2D p2 = {x_ + range_max_ * std::cos(current_angle),
                                      y_ + range_max_ * std::sin(current_angle)};
        turtlelib::Point2D center = {obstacles_x_[obs], obstacles_y_[obs]};

        double obs_distance = find_circle_intersection(p1, p2, center, obstacles_r_,
                                                           angle_min_ + laser_sample * angle_increment_, theta_);

        if (obs_distance < min_distance && obs_distance >= range_min_) {
            min_distance = obs_distance;
        }
      }

    // Update Lidar message
    if (min_distance < range_max_) {
        std::random_device rd;
        std::mt19937 gen(rd());

        // Mean of 0.0 and a variance of input_noise_
        std::normal_distribution<double> dist(0.0, std::sqrt(lidar_noise_));

        scan.ranges.at(laser_sample) = min_distance + dist(gen);
    }
}

// Publish Lidar scan
laser_scan_pub->publish(scan);
}

  /// \brief Finds the intersection of a line segment and a circle
  /// \param p1 The start of the line segment
  /// \param p2 The end of the line segment
  /// \param center The center of the circle
  /// \param radius The radius of the circle
  /// \param lidar_angle The angle of the lidar beam
  /// \param robot_angle The angle of the robot
  /// \return The distance to the intersection point
double find_circle_intersection(const turtlelib::Point2D & p1, const turtlelib::Point2D& p2,
                                const turtlelib::Point2D& center, double radius,
                                double lidar_angle, double robot_angle) {
    // translate coordinates so circle's center is at the origin
    turtlelib::Point2D p1_trans = {p1.x - center.x, p1.y - center.y};
    turtlelib::Point2D p2_trans = {p2.x - center.x, p2.y - center.y};
    double dx = p2_trans.x - p1_trans.x;
    double dy = p2_trans.y - p1_trans.y;
    double dr = std::sqrt(dx * dx + dy * dy);
    double D = p1_trans.x * p2_trans.y - p2_trans.x * p1_trans.y;

    double discriminant = radius * radius * dr * dr - D * D;

    // No intersection...
    if (discriminant < 0) {
        return -1.0;
    }

    // Determines intersection point to use
    int signDy;
    if (dy < 0) {
        signDy = -1;
    } else {
        signDy = 1;
    }

    // Calculate intersection points
    turtlelib::Point2D intersection1 = {
        (D * dy + signDy * dx * std::sqrt(discriminant)) / (dr * dr),
        (-D * dx + std::abs(dy) * std::sqrt(discriminant)) / (dr * dr)
    };

    turtlelib::Point2D intersection2 = {
        (D * dy - signDy * dx * std::sqrt(discriminant)) / (dr * dr),
        (-D * dx - std::abs(dy) * std::sqrt(discriminant)) / (dr * dr)
    };

    intersection1 = {center.x + intersection1.x, center.y + intersection1.y};
    intersection2 = {center.x + intersection2.x, center.y + intersection2.y};

    //  Lidar beam in world coordinates
    double lidar_x = std::cos(lidar_angle + robot_angle);
    double lidar_y = std::sin(lidar_angle + robot_angle);

    // Calculate dot products to determine whether the intersection points are in the direction of the Lidar beam
    double dot1 = (intersection1.x - p1.x) * lidar_x + (intersection1.y - p1.y) * lidar_y;
    double dot2 = (intersection2.x - p1.x) * lidar_x + (intersection2.y - p1.y) * lidar_y;

    // start to intersection distance
    double distance1 = measure_distance(p1.x, p1.y, intersection1.x, intersection1.y);
    double distance2 = measure_distance(p1.x, p1.y, intersection2.x, intersection2.y);

    // Determine the minimum distance 
    if (dot1 > 0.0 && (dot2 <= 0.0 || distance1 < distance2)) {
        return distance1;
    } else if (dot2 > 0.0 && (dot1 <= 0.0 || distance2 < distance1)) {
        return distance2;
    }

    return -1.0;
}

// ########## End_Citation [15] ##########

  /// \brief Resets the simulation to initial configuration.
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    timestep_ = 0;
    x_ = x0_;
    y_ = y0_;
    theta_ = theta0_;
  }

  /// \brief Moves the robot to a desired pose
  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    x_ = request->x;
    y_ = request->y;
    theta_ = request->theta;
  }

  /// \brief Creates four walls around the arena
  void create_walls_()
  {
    // The walls have a fixed height of 0.25m
    visualization_msgs::msg::Marker wall_mark_1;
    wall_mark_1.header.frame_id = "nusim/world";
    wall_mark_1.header.stamp = get_clock()->now();
    wall_mark_1.id = 1;
    wall_mark_1.type = visualization_msgs::msg::Marker::CUBE;
    wall_mark_1.action = visualization_msgs::msg::Marker::ADD;

    wall_mark_1.pose.position.x = 0.0;
    wall_mark_1.pose.position.y = arena_y_length_ / 2 + wall_thickness_ / 2;
    wall_mark_1.pose.position.z = 0.25 / 2;

    wall_mark_1.scale.x = arena_x_length_ + 2 * wall_thickness_;
    wall_mark_1.scale.y = wall_thickness_;
    wall_mark_1.scale.z = 0.25;

    wall_mark_1.color.r = 1.0f;
    wall_mark_1.color.a = 1.0f;

    wall_markers_array_.markers.push_back(wall_mark_1);

    visualization_msgs::msg::Marker wall_mark_2;
    wall_mark_2.header.frame_id = "nusim/world";
    wall_mark_2.header.stamp = get_clock()->now();
    wall_mark_2.id = 2;
    wall_mark_2.type = visualization_msgs::msg::Marker::CUBE;
    wall_mark_2.action = visualization_msgs::msg::Marker::ADD;

    wall_mark_2.pose.position.x = 0.0;
    wall_mark_2.pose.position.y = -(arena_y_length_ / 2 + wall_thickness_ / 2);
    wall_mark_2.pose.position.z = 0.25 / 2;

    wall_mark_2.scale.x = arena_x_length_ + 2 * wall_thickness_;
    wall_mark_2.scale.y = wall_thickness_;
    wall_mark_2.scale.z = 0.25;

    wall_mark_2.color.r = 1.0f;
    wall_mark_2.color.a = 1.0f;

    wall_markers_array_.markers.push_back(wall_mark_2);

    visualization_msgs::msg::Marker wall_mark_3;
    wall_mark_3.header.frame_id = "nusim/world";
    wall_mark_3.header.stamp = get_clock()->now();
    wall_mark_3.id = 3;
    wall_mark_3.type = visualization_msgs::msg::Marker::CUBE;
    wall_mark_3.action = visualization_msgs::msg::Marker::ADD;

    wall_mark_3.pose.position.x = arena_x_length_ / 2 + wall_thickness_ / 2;
    wall_mark_3.pose.position.y = 0.0;
    wall_mark_3.pose.position.z = 0.25 / 2;

    wall_mark_3.scale.x = wall_thickness_;
    wall_mark_3.scale.y = arena_y_length_ + 2 * wall_thickness_;
    wall_mark_3.scale.z = 0.25;

    wall_mark_3.color.r = 1.0f;
    wall_mark_3.color.a = 1.0f;

    wall_markers_array_.markers.push_back(wall_mark_3);

    visualization_msgs::msg::Marker wall_mark_4;
    wall_mark_4.header.frame_id = "nusim/world";
    wall_mark_4.header.stamp = get_clock()->now();
    wall_mark_4.id = 4;
    wall_mark_4.type = visualization_msgs::msg::Marker::CUBE;
    wall_mark_4.action = visualization_msgs::msg::Marker::ADD;

    wall_mark_4.pose.position.x = -(arena_x_length_ / 2 + wall_thickness_ / 2);
    wall_mark_4.pose.position.y = 0.0;
    wall_mark_4.pose.position.z = 0.25 / 2;

    wall_mark_4.scale.x = wall_thickness_;
    wall_mark_4.scale.y = arena_y_length_ + 2 * wall_thickness_;
    wall_mark_4.scale.z = 0.25;

    wall_mark_4.color.r = 1.0f;
    wall_mark_4.color.a = 1.0f;

    wall_markers_array_.markers.push_back(wall_mark_4);
  }

  /// \brief Creates static cylindrical obstacles in the simulation
  void create_obstacles()
  {
    if (obstacles_x_.size() != obstacles_y_.size()) {
      RCLCPP_ERROR(
        this->get_logger(), "Error: obstacles/x and obstacles/y should be the same length.");
      rclcpp::shutdown();
    }
    const auto num_markers = obstacles_x_.size();

    for (long unsigned int i = 0; i < num_markers; i++) {
      visualization_msgs::msg::Marker obs;
      obs.header.frame_id = "nusim/world";
      obs.header.stamp = get_clock()->now();
      obs.id = i;
      obs.type = visualization_msgs::msg::Marker::CYLINDER;
      obs.action = visualization_msgs::msg::Marker::ADD;

      obs.pose.position.x = obstacles_x_.at(i);
      obs.pose.position.y = obstacles_y_.at(i);
      obs.pose.position.z = 0.125;

      obs.scale.x = 2.0 * obstacles_r_;
      obs.scale.y = 2.0 * obstacles_r_;
      obs.scale.z = 0.25;

      obs.color.r = 1.0;
      obs.color.a = 1.0;
      obstacles_markers_array_.markers.push_back(obs);
    }
  }

  // Declare member variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr sensor_timer_;
  rclcpp::TimerBase::SharedPtr lidar_timer_;

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_service;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  visualization_msgs::msg::MarkerArray wall_markers_array_;
  visualization_msgs::msg::MarkerArray obstacles_markers_array_;

  // Publishers
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr red_sensor_data_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub;


  // Subscribers
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr red_wheel_sub;

  int timestep_;
  int rate_;
  double x0_;
  double y0_;
  double theta0_;
  double x_;
  double y_;
  double theta_;
  double arena_x_length_;
  double arena_y_length_;
  double wall_thickness_;
  std::vector<double> obstacles_x_;
  std::vector<double> obstacles_y_;
  double obstacles_r_;
  turtlelib::Wheels wheel_vel_;
  double motor_cmd_per_rad_sec_;
  double encoder_ticks_per_rad_;
  turtlelib::Wheels updated_wheel_pos_;
  turtlelib::Wheels prev_wheel_pos_{0.0, 0.0};
  nuturtlebot_msgs::msg::SensorData sensor_data_msg_;
  turtlelib::DiffDrive robot_;
  nav_msgs::msg::Path path_msg;
  double input_noise_;
  double slip_fraction_;
  double basic_sensor_variance_;
  double max_range_;
  double collision_radius_;
  double x_detect = 0.0;
  double y_detect = 0.0;
  double theta_detect = 0.0;
  double x_offset = 0.0;
  double y_offset = 0.0;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double time_increment_;
  double scan_time_;
  double range_min_;
  double range_max_;
  int num_samples_;
  double lidar_noise_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
