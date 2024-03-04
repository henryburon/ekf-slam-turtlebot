#include "turtlelib/se2d.hpp"
#include <cmath>
#include <iostream>
#include <sstream>

namespace turtlelib
{

    std::ostream &operator<<(std::ostream &os, const Twist2D &tw)
    {
        // Print the Twist2D in the format [w x y]
        return os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
    }

    std::istream &operator>>(std::istream &is, Twist2D &tw)
    {
        // Read the Twist2D in the format [w x y] or w x y
        char c1 = is.peek();

        if (c1 == '[')
        {
            is.get();
            is >> tw.omega >> tw.x >> tw.y;
            is.get();
        }
        else
        {
            is >> tw.omega >> tw.x >> tw.y;
        }
        return is;
    }

    // Identity transform
    Transform2D::Transform2D() : transf{0.0, 0.0}, rot{0.0} {}

    // Transformation that is pure translation
    Transform2D::Transform2D(Vector2D trans) : transf{trans.x, trans.y}, rot{0.0} {}

    // Transformation that is pure rotation, in radians
    Transform2D::Transform2D(double radians) : transf{0.0, 0.0}, rot{radians} {}

    // Transformation with a translational and rotational component
    Transform2D::Transform2D(Vector2D trans, double radians)
        : transf{trans.x, trans.y},
          rot{radians} {}

    // Apply a transformation to a 2D point
    Point2D Transform2D::operator()(Point2D p) const
    {
        double x_transf = std::cos(rot) * p.x - std::sin(rot) * p.y + transf.x;
        double y_transf = std::sin(rot) * p.x + std::cos(rot) * p.y + transf.y;
        return {x_transf, y_transf};
    }

    // Apply a transformation to a 2D vector
    Vector2D Transform2D::operator()(Vector2D v) const
    {
        double x_transf = std::cos(rot) * v.x - std::sin(rot) * v.y + transf.x;
        double y_transf = std::sin(rot) * v.x + std::cos(rot) * v.y + transf.y;
        return {x_transf, y_transf};
    }

    // Apply a transformation to a 2D twist (e.g. using the adjoint)
    Twist2D Transform2D::operator()(Twist2D v) const
    {
        double rot_transf = v.omega;
        double x_transf = transf.y * v.omega + std::cos(rot) * v.x - std::sin(rot) * v.y;
        double y_transf = -transf.x * v.omega + std::sin(rot) * v.x + std::cos(rot) * v.y;
        return {rot_transf, x_transf, y_transf};
    }

    // Invert the transformation
    Transform2D Transform2D::inv() const
    {
        // given Vector2D transf and double rot
        double rot_inv = -rot;
        double x_inv = -transf.x * std::cos(rot) - transf.y * std::sin(rot);
        double y_inv = -transf.y * std::cos(rot) + transf.x * std::sin(rot);
        return {Vector2D{x_inv, y_inv}, rot_inv};
    }

    Transform2D &Transform2D::operator*=(const Transform2D &rhs)
    {
        // '*=' is used to combine two transformations together
        // i.e. A *= B would mean do A then B, in sequence
        // Calculate how much the translation parts of A change when you apply rotation 'rhs' to it
        double x_A_moved = cos(rot) * rhs.transf.x - sin(rot) * rhs.transf.y;
        double y_A_moved = sin(rot) * rhs.transf.x + cos(rot) * rhs.transf.y;

        // Then update the translation to include those changes
        transf.x += x_A_moved;
        transf.y += y_A_moved;

        // And update the rotation...
        rot += rhs.rot;

        // Store the updated translation back in the object itself (i.e. returns updated A)
        return *this;
    }

    Vector2D Transform2D::translation() const
    {
        return {transf.x, transf.y};
    }

    double Transform2D::rotation() const
    {
        return rot;
    }

    std::ostream &operator<<(std::ostream &os, const Transform2D &tf)
    {
        // Prints a human readable version of the transform
        return os << "deg: " << rad2deg(tf.rot) << " x: " << tf.transf.x << " y: " << tf.transf.y;
    }

    std::istream &operator>>(std::istream &is, Transform2D &tf)
    {
        // Read a transformation from stdin
        std::string deg, x, y;
        double rot = 0.0;
        Vector2D tran{0.0, 0.0};

        char c1 = is.peek();

        if (c1 == 'd')
        {
            is >> deg;
            is >> rot;
            is >> x;
            is >> tran.x;
            is >> y;
            is >> tran.y;
        }
        else
        {
            is >> rot >> tran.x >> tran.y;
        }
        is.ignore(100, '\n');
        rot = deg2rad(rot);
        tf = Transform2D{tran, rot};
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D &rhs)
    {
        lhs *= rhs;
        return lhs;
    }

    // Don't have a test case for final: operator*

    Transform2D integrate_twist(Twist2D twist)
    {
        if (twist.omega == 0)
        {
            Transform2D T_bb_prime(Vector2D{twist.x, twist.y});
            return T_bb_prime;
        }
        else
        {
            // Find the center of rotation
            double x_s = (twist.y / twist.omega);
            double y_s = -(twist.x / twist.omega);

            Transform2D T_sb(Vector2D{x_s, y_s}); // s in the b frame (body to rotation)

            Transform2D T_ss_prime(twist.omega); // pure rotation in the new frame

            Transform2D T_bs = T_sb.inv();

            Transform2D T_bb_prime = T_bs * T_ss_prime * T_sb; // T_sb = T_s_prime_b_prime

            return T_bb_prime;

        }
    }

}