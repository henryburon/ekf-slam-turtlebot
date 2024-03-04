#include "turtlelib/geometry2d.hpp"
#include <cmath>
#include <iostream>

namespace turtlelib
{

    double normalize_angle(double rad)
    {
        // Normalize the angle to be within the range (-PI, PI]
        double normalizedAngle = rad - (ceil((rad + PI) / (2.0 * PI)) - 1.0) * 2.0 * PI;
        return normalizedAngle;
    }

    Vector2D normalize_vector(Vector2D v)
    {
        const auto mag = sqrt(v.x * v.x + v.y * v.y);
        if (mag != 0.0)
        {
            v.x /= mag;
            v.y /= mag;
        }
        return v;
    }

    std::ostream &operator<<(std::ostream &os, const Point2D &p)
    {
        // Outputs a 2-dimensional point as [x y]
        return os << "[" << p.x << " " << p.y << "]";
    }

    std::istream &operator>>(std::istream &is, Point2D &p)
    {
        // Read vectors entered as [x y] or x y
        char ch;
        // Extracts characters from stream
        // This if-statement is designed to work with the multiple input types
        // So, if successful extraction from the input stream...
        if (is >> ch)
        {
            // First, check if the character is a '[
            if (ch == '[')
            {
                // If the input starts with '[', read x and y values, then expect a closing ']'
                is >> p.x >> p.y;
                is >> ch; // Read the expected ']'
                if (ch != ']')
                {
                    // If the next character is not ']', set the stream to a fail state
                    is.setstate(std::ios::failbit);
                }
            }
            else
            {
                // If the input does not start with '[', put the character back, then read x and y
                is.putback(ch);
                is >> p.x >> p.y;
            }
        }
        return is;
    }

    Vector2D operator-(const Point2D &head, const Point2D &tail)
    {
        // Subtracting one point from another yields a vector
        return {head.x - tail.x, head.y - tail.y};
    }

    Point2D operator+(const Point2D &tail, const Vector2D &disp)
    {
        // Adding a vector to a point yields a new point displaced by the vector
        return {tail.x + disp.x, tail.y + disp.y};
    }

    std::ostream &operator<<(std::ostream &os, const Vector2D &v)
    {
        // Outputs a 2-dimensional vector as [x y]
        return os << "[" << v.x << " " << v.y << "]";
    }

    std::istream &operator>>(std::istream &is, Vector2D &v)
    {
        // Read vectors entered as [x y] or x y
        char ch;
        // Extracts characters from stream
        // This if-statement is designed to work with the multiple input types
        // So, if successful extraction from the input stream...
        if (is >> ch)
        {
            // First, check if the character is a '[
            if (ch == '[')
            {
                // If the input starts with '[', read x and y values, then expect a closing ']'
                is >> v.x >> v.y;
                is >> ch; // Read the expected ']'
                if (ch != ']')
                {
                    // If the next character is not ']', set the stream to a fail state
                    is.setstate(std::ios::failbit);
                }
            }
            else
            {
                // If the input does not start with '[', put the character back, then read x and y
                is.putback(ch);
                is >> v.x >> v.y;
            }
        }
        return is;
    }

    Vector2D & Vector2D::operator+=(const Vector2D & rhs)
    {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator-=(const Vector2D & rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator*=(const double & scalar)
    {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    Vector2D operator+(const Vector2D &lhs, const Vector2D &rhs)
    {
        Vector2D result;
        result.x = lhs.x + rhs.x;
        result.y = lhs.y + rhs.y;
        return result;
    }

    Vector2D operator-(const Vector2D &lhs, const Vector2D &rhs)
    {
        Vector2D result;
        result.x = lhs.x - rhs.x;
        result.y = lhs.y - rhs.y;
        return result;
    }

    Vector2D operator*(const Vector2D &lhs, const double & scalar)
    {
        Vector2D result;
        result.x = lhs.x * scalar;
        result.y = lhs.y * scalar;
        return result;
    }

    double dot(Vector2D v1, Vector2D v2)
    {
        double result = v1.x * v2.x + v1.y * v2.y;
        return result;
    }

    double magnitude(Vector2D v)
    {
        double result = std::sqrt(v.x * v.x + v.y * v.y);
        return result;
    }

    double angle(Vector2D v1, Vector2D v2)
    {
        double dot12 = dot(v1, v2);
        double mag12 = magnitude(v1) * magnitude(v2);
        double result = acos(dot12/mag12);
        return result;
    }

}


