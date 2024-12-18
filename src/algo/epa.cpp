#include "nbody/math/vector_math.hpp"
#include "nbody/algo/epa.hpp"
#include <cmath>
#include <limits>
#include <iostream>
#include <optional>
#include <cassert>

static double edgeDistance(const Vector &a, const Vector &b, Vector &normal) {
    Vector e = b - a;
    normal = Vector(e.y, -e.x).normalized();
    double dist = normal.dotProduct(a);
    if (dist < 0) {
        normal.x = -normal.x;
        normal.y = -normal.y;
        dist = -dist;
    }
    return dist;
}

std::optional<EPAResult> EPA(const ShapeData &A, const ShapeData &B, const Simplex &simplex) {
    std::vector<Vector> poly = simplex.points;

    // Expect a triangle from GJK if intersection found
    assert(poly.size() == 3 && "GJK should return a simplex of 3 points at intersection.");

    // Check for degenerate polygon (collinear points)
    {
        Vector ab = poly[1] - poly[0];
        Vector ac = poly[2] - poly[0];
        double crossVal = ab.cross(ac);
        if (std::fabs(crossVal) < 1e-14) {
            std::cerr << "Warning: EPA degenerate simplex (collinear points). Returning no penetration." << std::endl;
            return std::nullopt;
        }
    }

    // Ensure CCW order
    {
        double crossVal = (poly[1].x - poly[0].x)*(poly[2].y - poly[0].y)
                        - (poly[1].y - poly[0].y)*(poly[2].x - poly[0].x);
        if (crossVal < 0) {
            std::reverse(poly.begin(), poly.end());
        }
    }

    const int MAX_EPA_ITER = 100;
    for (int iter=0; iter<MAX_EPA_ITER; iter++) {
        double closestDist = std::numeric_limits<double>::max();
        int closestEdge = -1;
        Vector edgeNormal;

        // Find closest edge to origin
        for (int i=0; i<(int)poly.size(); i++) {
            int j=(i+1)%poly.size();
            Vector normal;
            double dist = edgeDistance(poly[i], poly[j], normal);
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
        Vector p = supportMinkowski(A,B,edgeNormal);
        double d = p.dotProduct(edgeNormal);

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