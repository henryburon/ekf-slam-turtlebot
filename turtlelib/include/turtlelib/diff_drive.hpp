#ifndef TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
#define TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Models the kinematics of a differential drive robot

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    /// \brief represent the position of a diff drive robot's wheels
    struct Wheels
    {
        /// \brief the left wheel's position (rad)
        double phi_l = 0.0;

        /// \brief the right wheel's position (rad)
        double phi_r = 0.0;
    };

    /// \brief Structure to represent the configuration of a robot in 2D space
    struct RobotConfig
    {
        /// \brief Robot's orientation in radians, relative to the x-axis
        double theta = 0.0;
        
        /// \brief the robot's x-coordinate in the world frame
        double x = 0.0;

        /// \brief the robot's y-coordinate in the world frame
        double y = 0.0;

    };
    
    /// @brief  \brief Models a differential drive robot
    class DiffDrive
    {
        private:

        /// \brief Distace between the centers of the wheels
        double wheel_track;

        /// \brief Radius of each wheel
        double wheel_radius;

        /// \brief Current position of the wheels
        Wheels wheels;

        /// \brief Current configuration of the robot
        RobotConfig q;

        public:

        /// \brief Default constructor. Initializes a DiffDrive object with default parameters
        DiffDrive();

        /// \brief create a DiffDrive object with specific attributes
        /// \param wheel_track - Distance between the centers of the wheels
        /// \param wheel_radius - Radius of the wheels
        /// \param wheels - Initial position of the robot's wheels
        /// \param q - Initial configuration of the robot in the world frame
        DiffDrive(double wheel_track, double wheel_radius, Wheels wheels, RobotConfig q);

        /// \brief Retrieves the current wheel positions
        /// \return A Wheels struct containing the current positions of the wheels
        Wheels get_wheels() const;

        /// \brief Retrieves the current robot configuration
        /// \brief A RobotConfig object representing the robot's current position and orientation
        RobotConfig get_robot_config() const;

        /// \brief Sets new positions for the robot's wheels
        /// \param new_wheels - new wheels position
        void set_wheels(Wheels new_wheels);

        /// \brief Sets a new configuration for the robot
        /// \param q_new - A RobotConfig struct representing the new position and orientation of the robot
        void set_robot_config(RobotConfig q_new);

        /// \brief Updates the robot's configuration based on a change in wheel position
        /// \param delta_wheels A Wheels struct representing the change in position of the left and right wheel
        void forward_kinematic_update(Wheels delta_wheels);

        /// \brief Calculates the required wheel movement to achieve a specified twist
        /// \param twist - A Twist2D struct representing the desired twist of the robot
        /// \returns A Wheels struct containing the wheel movements required to achieve the specified twist
        /// \throws std::logic_error If the desired twist cannot be achieved without wheel slippage
        Wheels inverse_kinematics(Twist2D twist);





    };
}

#endif