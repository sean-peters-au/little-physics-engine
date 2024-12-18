#ifndef POLYGON_SHAPE_HPP
#define POLYGON_SHAPE_HPP

#include <vector>
#include <cmath>
#include "nbody/components/basic.hpp"
#include "nbody/math/vector_math.hpp"

struct PolygonShape {
    Components::ShapeType type;
    std::vector<Vector> vertices; // local space
};

struct CircleShape {
    double radius;
};

// Given a polygon, a direction vector, a position, and an angle,
// this function returns the world-space vertex of the polygon 
// that is furthest in the given direction.
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

// For a circle, we just move along the direction by its radius
inline Vector supportCircle(double radius, const Position &pos, double angle, const Vector &direction) {
    (void)angle; // angle doesn't affect a circle
    double len = std::sqrt(direction.x*direction.x+direction.y*direction.y);
    Vector dirNorm = direction;
    if (len > 1e-9) {
        dirNorm.x /= len;
        dirNorm.y /= len;
    }
    return Vector(pos.x + dirNorm.x*radius, pos.y + dirNorm.y*radius);
}

// ShapeData as before
struct ShapeData {
    bool isCircle;
    double radius;
    PolygonShape poly;
    Position pos;
    double angle;
};

// Minkowski support function: gets support from A in d, support from B in -d, and subtracts.
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