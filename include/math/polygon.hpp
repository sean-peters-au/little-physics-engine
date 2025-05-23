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

#pragma once

#include <cmath>
#include <random>
#include <vector>

#include "entities/entity_components.hpp"
#include "math/vector_math.hpp"

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

/**
 * @brief Builds a regular polygon shape
 * 
 * This function creates a regular polygon with a specified number of sides
 * and a given size (radius). The polygon is oriented counter-clockwise and
 * its vertices are stored in local space coordinates.
 * 
 * @param sides Number of sides in the polygon
 * @param sz Size (radius) of the polygon
 * @return PolygonShape The regular polygon shape
 */
inline PolygonShape buildRegularPolygon(int sides, double sz)
{
    PolygonShape poly;
    poly.type = Components::ShapeType::Polygon;
    double const angleStep = 2.0 * M_PI / static_cast<double>(sides);
    for (int i = 0; i < sides; i++) {
        double const angle = i * angleStep;
        double const x = sz * std::cos(angle);
        double const y = -sz * std::sin(angle);
        poly.vertices.emplace_back(x, y);
    }
    return poly;
}

/*
 * @brief Build a random convex polygon
 * 
 * This function creates a random convex polygon with 3-7 sides
 * and a size (radius) that ranges from half to the specified size.
 * The polygon is oriented counter-clockwise and its vertices are stored
 * in local space coordinates.
 * 
 * @param gen Random number generator
 * @param sz Maximum size (radius) of the polygon
 * @return PolygonShape The random convex polygon shape
*/
inline PolygonShape buildRandomConvexPolygon(std::default_random_engine &gen, double sz)
{
    PolygonShape poly;
    poly.type = Components::ShapeType::Polygon;

    std::uniform_int_distribution<int> sideDist(3, 7);
    std::uniform_real_distribution<double> radiusDist(0.5 * sz, sz);

    int const sides = sideDist(gen);
    double const angleStep = 2.0 * M_PI / static_cast<double>(sides);

    double angle = 0.0;
    for(int i = 0; i < sides; i++) {
        double const r = radiusDist(gen);
        double const x = r * std::cos(angle);
        double const y = -r * std::sin(angle);
        poly.vertices.emplace_back(x, y);
        angle += angleStep;
    }
    return poly;
}


/**
 * @brief Builds a random polygon shape
 * 
 * This function creates a random polygon with a variable number of sides
 * and a size (radius) that ranges from half to the specified size.
 * The polygon is oriented counter-clockwise and its vertices are stored
 * in local space coordinates.
 * 
 * @param gen Random number generator
 * @param sz Maximum size (radius) of the polygon
 * @return PolygonShape The random polygon shape
 */
inline PolygonShape buildRandomPolygon(std::default_random_engine &gen, double sz)
{
    PolygonShape poly;
    poly.type = Components::ShapeType::Polygon;

    // Generate between 5-10 sides for more interesting shapes
    std::uniform_int_distribution<int> sideDist(5, 10);
    int vertices = sideDist(gen);
    
    // Generate random points in a box of size sz×sz
    std::vector<Vector> points;
    std::uniform_real_distribution<double> coordDist(-sz, sz);
    
    for (int i = 0; i < vertices; i++) {
        double x = coordDist(gen);
        double y = coordDist(gen);
        points.push_back(Vector(x, y));
    }
    
    // Find centroid
    Vector centroid(0, 0);
    for (const auto& p : points) {
        centroid.x += p.x;
        centroid.y += p.y;
    }
    centroid.x /= points.size();
    centroid.y /= points.size();
    
    // Sort points by angle around the centroid (counter-clockwise in screen coords)
    std::sort(points.begin(), points.end(), [centroid](const Vector& a, const Vector& b) {
        return std::atan2(-(a.y - centroid.y), a.x - centroid.x) < 
               std::atan2(-(b.y - centroid.y), b.x - centroid.x);
    });
    
    // Add the sorted points to the polygon
    for (const auto& p : points) {
        poly.vertices.push_back(p);
    }
    
    return poly;
}

/**
 * @brief Calculates the moment of inertia for a polygon
 * 
 * This function computes the moment of inertia for a polygon based on its
 * vertices and mass. It uses the parallel axis theorem to shift the
 * vertices to the center of mass.
 * 
 * @param vertices Vector of vertices in local space coordinates
 * @param mass Mass of the polygon
 * @return double Moment of inertia
 */
inline double calculatePolygonInertia(const std::vector<Vector>& vertices, double mass) {
    double numerator = 0.0;
    double denominator = 0.0;
    
    int const n = vertices.size();
    for (int i = 0; i < n; i++) {
        int const j = (i + 1) % n;
        double const cross = vertices[i].cross(vertices[j]);
        numerator += cross * (vertices[i].dotProduct(vertices[i]) + 
                              vertices[i].dotProduct(vertices[j]) + 
                              vertices[j].dotProduct(vertices[j]));
        denominator += cross;
    }
    
    // For a polygon with uniform density
    return (mass * numerator) / (6.0 * denominator);
}
