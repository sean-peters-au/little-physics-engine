/**
 * @file polygon.hpp
 * @brief Geometric primitives and support functions for collision detection
 *
 * This file provides geometric primitives (polygons and circles) and support functions
 * used in collision detection algorithms (GJK/EPA). It implements:
 * - Basic shape definitions for polygons and circles
 * - Support functions for Minkowski difference calculations
 * - World-space transformation utilities
 * 
 * The support functions are critical components of the GJK/EPA algorithms,
 * finding the furthest point of a shape in a given direction.
 */

#ifndef POLYGON_SHAPE_HPP
#define POLYGON_SHAPE_HPP

#include <vector>
#include <cmath>
#include "nbody/components/basic.hpp"
#include "nbody/math/vector_math.hpp"

/**
 * @brief Represents a convex polygon in 2D space
 * 
 * Stores vertices in local space (relative to shape center). Vertices should be
 * ordered counter-clockwise for consistent collision detection.
 */
struct PolygonShape {
    Components::ShapeType type;         ///< Type identifier for the shape
    std::vector<Vector> vertices;       ///< Vertices in local space coordinates
};

/**
 * @brief Represents a circle in 2D space
 */
struct CircleShape {
    double radius;                      ///< Radius of the circle
};

/**
 * @brief Finds the furthest point of a polygon in a given direction
 * 
 * Implements the support function for polygons used in GJK/EPA algorithms.
 * Transforms local vertices to world space and finds the one that projects
 * furthest along the given direction.
 * 
 * @param poly The polygon to query
 * @param direction Direction vector to project along
 * @param pos World position of the polygon
 * @param angle Rotation angle in radians
 * @return Vector The world-space vertex furthest in the given direction
 */
inline Vector supportPolygon(const PolygonShape &poly, 
                           const Vector &direction, 
                           const Position &pos, 
                           double angle) 
{
    double c = std::cos(angle);
    double s = std::sin(angle);

    double bestProj = -1e9;
    Vector best;
    for (auto &lv : poly.vertices) {
        // Transform local vertex by angle and position
        double wx = pos.x + (lv.x*c - lv.y*s);
        double wy = pos.y + (lv.x*s + lv.y*c);
        double proj = wx*direction.x + wy*direction.y;
        if (proj > bestProj) {
            bestProj = proj;
            best.x=wx; best.y=wy;
        }
    }
    return best;
}

/**
 * @brief Finds the furthest point of a circle in a given direction
 * 
 * Implements the support function for circles used in GJK/EPA algorithms.
 * For circles, this is simply the center plus the normalized direction
 * vector scaled by the radius.
 * 
 * @param radius Circle radius
 * @param pos World position of the circle center
 * @param angle Rotation angle (unused for circles)
 * @param direction Direction vector to project along
 * @return Vector The world-space point furthest in the given direction
 */
inline Vector supportCircle(double radius, 
                          const Position &pos, 
                          double angle, 
                          const Vector &direction) {
    (void)angle; // angle doesn't affect a circle
    double len = std::sqrt(direction.x*direction.x+direction.y*direction.y);
    Vector dirNorm = direction;
    if (len > 1e-9) {
        dirNorm.x /= len;
        dirNorm.y /= len;
    }
    return Vector(pos.x + dirNorm.x*radius, pos.y + dirNorm.y*radius);
}

/**
 * @brief Combined shape data for collision detection
 * 
 * Unified structure that can represent either a polygon or circle,
 * along with its transform information (position and rotation).
 */
struct ShapeData {
    bool isCircle;                      ///< True if shape is a circle, false for polygon
    double radius;                      ///< Radius (only used if isCircle is true)
    PolygonShape poly;                  ///< Polygon data (only used if isCircle is false)
    Position pos;                       ///< World position of the shape
    double angle;                       ///< Rotation angle in radians
};

/**
 * @brief Calculates the support point of the Minkowski difference of two shapes
 * 
 * This function is central to the GJK/EPA algorithms, computing the furthest point
 * of shape A minus the furthest point of shape B in the opposite direction.
 * 
 * @param A First shape
 * @param B Second shape
 * @param d Direction vector for support calculation
 * @return Vector Support point in Minkowski difference space
 */
inline Vector supportMinkowski(const ShapeData &A, const ShapeData &B, const Vector &d) {
    Vector pA = A.isCircle ?
        supportCircle(A.radius, A.pos, A.angle, d) :
        supportPolygon(A.poly, d, A.pos, A.angle);

    Vector negd(-d.x, -d.y);
    Vector pB = B.isCircle ?
        supportCircle(B.radius, B.pos, B.angle, negd) :
        supportPolygon(B.poly, negd, B.pos, B.angle);

    return Vector(pA.x - pB.x, pA.y - pB.y);
}

#endif