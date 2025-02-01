/**
 * @file vector_math.hpp
 * @brief 2D vector and position mathematics library
 *
 * This file provides fundamental 2D geometric primitives and operations:
 * - Vector class for direction and magnitude calculations
 * - Position class for point locations in 2D space
 * - Geometric operations (dot product, cross product, rotations)
 * - Utility functions for common mathematical operations
 */

#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

// Forward declarations
class Vector;

/**
 * @brief Constants for floating-point comparisons
 */
constexpr double EPSILON = 1e-9;  ///< Threshold for floating point equality tests

/**
 * @brief Utility function for safe square root computation
 * 
 * @param d Input value
 * @return double Square root of input, warns if input is negative
 */
double my_sqrt(double d);

/**
 * @brief Compares two doubles for approximate equality
 * 
 * @param a First value
 * @param b Second value
 * @param epsilon Maximum allowed difference
 * @return true if |a-b| < epsilon
 */
bool nearlyEqual(double a, double b, double epsilon=EPSILON);

/**
 * @brief Represents a 2D point in space
 * 
 * Position class is used for absolute locations in 2D space.
 * Supports basic arithmetic and conversion to/from Vector.
 */
class Position {
public:
    double x;  ///< X coordinate
    double y;  ///< Y coordinate

    /** @brief Constructs a Position at (0,0) */
    Position();
    
    /**
     * @brief Constructs a Position at specified coordinates
     * @param x X coordinate
     * @param y Y coordinate
     */
    Position(double x, double y);
    
    /** @brief Converts Position to Vector */
    operator Vector() const;

    /**
     * @brief Adds two positions
     * @param p Position to add
     * @return New position at component-wise sum
     */
    Position operator+(const Position& b) const;
    
    /**
     * @brief Subtracts two positions
     * @param p Position to subtract
     * @return New position at component-wise difference
     */
    Position operator-(const Position& b) const;
    
    /**
     * @brief Scales position by scalar value
     * @param scalar Scale factor
     * @return New position with coordinates multiplied by scalar
     */
    Position operator*(double scalar) const;
    
    /**
     * @brief Divides position by scalar value
     * @param scalar Divisor
     * @return New position with coordinates divided by scalar
     */
    Position operator/(double scalar) const;

    /**
     * @brief Calculates Euclidean distance to another position
     * @param p Target position
     * @return Distance between positions
     */
    double dist(const Position& p) const;

    /**
     * @brief Adds another position to this one
     * @param p Position to add
     * @return Reference to this position
     */
    Position& operator+=(const Position& p);
    
    /**
     * @brief Subtracts another position from this one
     * @param p Position to subtract
     * @return Reference to this position
     */
    Position& operator-=(const Position& p);
};

/**
 * @brief Represents a 2D vector with direction and magnitude
 * 
 * Vector class provides comprehensive 2D vector operations for
 * physics calculations and geometric algorithms.
 */
class Vector {
public:
    double x;  ///< X component
    double y;  ///< Y component

    /** @brief Constructs a zero vector (0,0) */
    Vector();
    
    /**
     * @brief Constructs a vector with given components
     * @param x X component
     * @param y Y component
     */
    Vector(double x, double y);
    
    /**
     * @brief Constructs a vector from a position
     * @param p Position to convert
     */
    Vector(const Position& p);
    
    /** @brief Converts Vector to Position */
    operator Position() const;

    /** @brief Returns negation of this vector */
    Vector operator-() const;
    
    /**
     * @brief Adds two vectors
     * @param v Vector to add
     * @return Sum vector
     */
    Vector operator+(const Vector& b) const;
    
    /**
     * @brief Subtracts two vectors
     * @param v Vector to subtract
     * @return Difference vector
     */
    Vector operator-(const Vector& b) const;
    
    /**
     * @brief Scales vector by scalar value
     * @param scalar Scale factor
     * @return Scaled vector
     */
    Vector operator*(double scalar) const;
    
    /**
     * @brief Divides vector by scalar value
     * @param scalar Divisor
     * @return Divided vector
     */
    Vector operator/(double scalar) const;

    /**
     * @brief Rotates vector 90 degrees clockwise
     * @return Rotated vector
     */
    Vector rotate() const;
    
    /**
     * @brief Scales vector to specified length
     * @param length Target length
     * @return Vector with same direction but new length
     */
    Vector scale(double length) const;
    
    /** @brief Returns vector magnitude */
    double length() const;
    
    /**
     * @brief Calculates dot product with another vector
     * @param v Other vector
     * @return Dot product value
     */
    double dotProduct(const Vector& v) const;
    
    /**
     * @brief Calculates 2D cross product with another vector
     * @param other Other vector
     * @return Cross product value (z-component)
     */
    double cross(const Vector &other) const;
    
    /** @brief Returns perpendicular vector (rotated 90 degrees) */
    Vector perp() const;
    
    /** @brief Returns normalized vector (length = 1) */
    Vector normalized() const;
    
    /**
     * @brief Calculates angle between this and another vector
     * @param other Other vector
     * @return Angle in radians
     */
    double angleBetween(const Vector &other) const;
    
    /**
     * @brief Rotates vector by specified angle
     * @param angle Rotation angle in radians
     * @return Rotated vector
     */
    Vector rotateByAngle(double angle) const;
    
    /**
     * @brief Projects vector length onto another vector
     * @param onto Vector to project onto
     * @return Projected length
     */
    double projectLength(const Vector &onto) const;
    
    /**
     * @brief Projects vector onto another vector
     * @param onto Vector to project onto
     * @return Projected vector
     */
    Vector projectOnto(const Vector &onto) const;

    /**
     * @brief Adds another vector to this one
     * @param v Vector to add
     * @return Reference to this vector
     */
    Vector& operator+=(const Vector& v);
    
    /**
     * @brief Subtracts another vector from this one
     * @param v Vector to subtract
     * @return Reference to this vector
     */
    Vector& operator-=(const Vector& v);
};

/**
 * @brief Finds closest point on line segment to a point
 * 
 * @param a Start point of line segment
 * @param b End point of line segment
 * @param p Point to find closest position to
 * @return Vector Position of closest point on line segment ab
 */
Vector closestPointOnLine(const Vector &a, const Vector &b, const Vector &p);

#endif