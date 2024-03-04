#include "turtlelib/diff_drive.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>


namespace turtlelib
{
    
    DiffDrive::DiffDrive(): // track, radius, wheels, config
    DiffDrive(0.16, 0.33, {0.0, 0.0}, {0.0, 0.0, 0.0}){}

    DiffDrive::DiffDrive(double wheel_track, double wheel_radius, Wheels wheels, RobotConfig q)
    : wheel_track(wheel_track), wheel_radius(wheel_radius), wheels(wheels), q(q)
    {

    }

    Wheels DiffDrive::get_wheels() const
    {
        return wheels;
    }

    RobotConfig DiffDrive::get_robot_config() const
    {
        return q;
    }

    void DiffDrive::set_wheels(Wheels new_wheels)
    {
        wheels.phi_r = new_wheels.phi_r;
        wheels.phi_l = new_wheels.phi_l;
    }

    void DiffDrive::set_robot_config(RobotConfig q_new)
    {
        q.theta = q_new.theta;
        q.x = q_new.x;
        q.y = q_new.y;
    }

// ########## Begin_Citation [7] ##########

    void DiffDrive::forward_kinematic_update(Wheels delta_wheels)
    {
        // [1] Get the body twist Vb

        Twist2D Vb;
        Vb.omega = ((wheel_radius / (2.0 * (wheel_track/2.0))) * (delta_wheels.phi_r - delta_wheels.phi_l));
        Vb.x = (wheel_radius / 2.0) * (delta_wheels.phi_l + delta_wheels.phi_r);

        // [2] Find the body transformation from the twist
        // This expresses the new chassis frame, b_p, 
        // relative to the initial frame, b.

        Transform2D T_bb_p = integrate_twist(Vb);

        // Extract the transformation values from
        // b to b_p

        double d_qb_p_theta = T_bb_p.rotation();
        double d_qb_x = T_bb_p.translation().x;
        double d_qb_y = T_bb_p.translation().y;
        
        // [3] Transform dqb in {b} to dq in {s}

        double dqtheta = d_qb_p_theta;
        double dqx = std::cos(q.theta) * d_qb_x - std::sin(q.theta) * d_qb_y;
        double dqy = std::sin(q.theta) * d_qb_x + std::cos(q.theta) * d_qb_y;

        // [4] Update robot config

        q.theta += dqtheta;
        q.theta = normalize_angle(q.theta);
        q.x += dqx;
        q.y += dqy;

        // Update wheel positions as well?
        wheels.phi_l += delta_wheels.phi_l;
        wheels.phi_r += delta_wheels.phi_r;
    }


// ########## End_Citation [7] ##########

// ########## Begin_Citation [12] ##########

    Wheels DiffDrive::inverse_kinematics(Twist2D twist)
    {

        if (twist.y != 0.0)
        {
            throw std::logic_error("Twist cannot be accomplished without the wheels slipping.");
        }
        
        Wheels required_wheels;
        
        // [5]
        required_wheels.phi_l = ((-(wheel_track/2.0) * twist.omega + twist.x) / wheel_radius);

        // [6]
        required_wheels.phi_r = (((wheel_track/2.0) * twist.omega + twist.x) / wheel_radius);

        return required_wheels;
    }
// ########## End_Citation [12] ##########

}

