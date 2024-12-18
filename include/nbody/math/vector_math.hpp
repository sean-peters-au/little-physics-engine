#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

// Forward declarations
class Vector;

/**
 * Constants
 */
constexpr double EPSILON = 1e-9;  /** Threshold for floating point comparisons */

/**
 * Utility Functions
 */
/** Safely computes square root, prints warning if input is negative */
double my_sqrt(double d);

/** Returns true if two doubles are nearly equal within epsilon */
bool nearlyEqual(double a, double b, double epsilon=EPSILON);

/**
 * Position Class
 * Represents a 2D point in space
 */
class Position {
public:
    double x;
    double y;

    // Constructors
    /** Constructs a Position at (0,0) */
    Position();
    /** Constructs a Position at specified coordinates */
    Position(double x, double y);
    
    // Type conversion
    /** Converts Position to Vector */
    operator Vector() const;

    // Basic arithmetic operators (immutable)
    /** Returns sum of two positions */
    Position operator+(const Position& p) const;
    /** Returns difference between two positions */
    Position operator-(const Position& p) const;
    /** Returns position scaled by scalar */
    Position operator*(double scalar) const;
    /** Returns position divided by scalar */
    Position operator/(double scalar) const;

    // Geometric operations (immutable)
    /** Returns Euclidean distance to another position */
    double dist(const Position& p) const;
};

/**
 * Vector Class
 * Represents a 2D direction and magnitude
 */
class Vector {
public:
    double x;
    double y;

    // Constructors
    /** Constructs a zero vector */
    Vector();
    /** Constructs a vector with given components */
    Vector(double x, double y);
    /** Constructs a vector from a position */
    Vector(const Position& p);
    
    // Type conversion
    /** Converts Vector to Position */
    operator Position() const;

    // Basic arithmetic operators (immutable)
    /** Returns the negative of a vector */
    Vector operator-() const;
    /** Returns sum of two vectors */
    Vector operator+(const Vector& v) const;
    /** Returns difference between two vectors */
    Vector operator-(const Vector& v) const;
    /** Returns vector scaled by scalar */
    Vector operator*(double scalar) const;
    /** Returns vector divided by scalar */
    Vector operator/(double scalar) const;

    // Geometric operations (immutable)
    /** Returns a vector rotated 90 degrees clockwise */
    Vector rotate() const;
    /** Returns a vector scaled to specified length, preserving direction */
    Vector scale(double length) const;
    /** Returns vector magnitude */
    double length() const;
    /** Returns dot product with another vector */
    double dotProduct(const Vector& v) const;
    /** Returns cross product with another vector */
    double cross(const Vector &other) const;
    /** Returns a perpendicular vector */
    Vector perp() const;
    /** Returns a normalized vector to unit length */
    Vector normalized() const;
    /** Returns angle between this and another vector in radians */
    double angleBetween(const Vector &other) const;
    /** Returns this vector rotated by specified angle (in radians) */
    Vector rotateByAngle(double angle) const;
    /** Returns projection length onto another vector */
    double projectLength(const Vector &onto) const;
    /** Returns projection of this onto another vector */
    Vector projectOnto(const Vector &onto) const;
};

/**
 * Free Functions - Vector Operations
 */
/** Returns closest point on line segment ab to point p */
Vector closestPointOnLine(const Vector &a, const Vector &b, const Vector &p);

#endif