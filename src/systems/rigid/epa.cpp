#include "nbody/math/vector_math.hpp"
#include "nbody/systems/rigid/epa.hpp"

#include <cmath>
#include <limits>
#include <iostream>
#include <optional>
#include <cassert>

/**
 * @brief Calculates the perpendicular distance from a line segment to the origin
 * 
 * @param a First point of the edge
 * @param b Second point of the edge
 * @param[out] normal The normalized perpendicular vector to the edge, pointing away from origin
 * @return double The perpendicular distance from the edge to the origin
 * 
 * @note The normal is always oriented to point away from the origin
 */
static double edgeDistance(const Vector &a, const Vector &b, Vector &normal) {
    Vector const e = b - a;
    normal = Vector(e.y, -e.x).normalized();
    double dist = normal.dotProduct(a);
    if (dist < 0) {
        normal.x = -normal.x;
        normal.y = -normal.y;
        dist = -dist;
    }
    return dist;
}

std::optional<EPAResult> EPA(const ShapeData &a, const ShapeData &b, const Simplex &simplex) {
    std::vector<Vector> poly = simplex.points;

    // Expect a triangle from GJK if intersection found
    assert(poly.size() == 3 && "GJK should return a simplex of 3 points at intersection.");

    // Check for degenerate polygon (collinear points)
    {
        Vector const ab = poly[1] - poly[0];
        Vector const ac = poly[2] - poly[0];
        double const crossVal = ab.cross(ac);
        if (std::fabs(crossVal) < 1e-14) {
            std::cerr << "Warning: EPA degenerate simplex (collinear points). Returning no penetration." << std::endl;
            return std::nullopt;
        }
    }

    // Ensure CCW order
    {
        double const crossVal = (poly[1].x - poly[0].x)*(poly[2].y - poly[0].y)
                        - (poly[1].y - poly[0].y)*(poly[2].x - poly[0].x);
        if (crossVal < 0) {
            std::reverse(poly.begin(), poly.end());
        }
    }

    const int maxEpaIter = 100;
    for (int iter=0; iter<maxEpaIter; iter++) {
        double closestDist = std::numeric_limits<double>::max();
        int closestEdge = -1;
        Vector edgeNormal;

        // Find closest edge to origin
        for (int i=0; i<static_cast<int>(poly.size()); i++) {
            int const j=(i+1)%poly.size();
            Vector normal;
            double const dist = edgeDistance(poly[i], poly[j], normal);
            if (dist < closestDist) {
                closestDist=dist;
                closestEdge=i;
                edgeNormal=normal;
            }
        }

        if (closestEdge<0) {
            std::cerr << "Warning: EPA no closest edge found. Returning no penetration." << std::endl;
            return std::nullopt;
        }

        // Get support in edgeNormal direction
        Vector const p = supportMinkowski(a,b,edgeNormal);
        double const d = p.dotProduct(edgeNormal);

        // If no improvement
        if (d - closestDist < EPSILON) {
            EPAResult res;
            res.normal = edgeNormal;
            res.penetration = d;
            return res;
        }

        // Insert new point in polygon
        poly.insert(poly.begin() + ((closestEdge+1)%poly.size()), p);
    }

    // If no convergence
    std::cerr << "Warning: EPA exceeded max iterations. Returning no penetration." << std::endl;
    return std::nullopt;
}