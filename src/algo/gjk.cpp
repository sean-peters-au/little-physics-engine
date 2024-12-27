#include "nbody/algo/gjk.hpp"
#include "nbody/math/polygon.hpp"

#include <cmath>
#include <limits>
#include <iostream>
#include <cassert>

static bool handleSimplex(Simplex &simplex, Vector &direction) {
    size_t count = simplex.points.size();
    assert(count > 0 && count <= 3 && "Simplex should have 1 to 3 points.");

    if (count == 2) {
        Vector a = simplex.points[1];
        Vector b = simplex.points[0];
        Vector ab = b - a;
        Vector ao = -a;

        // If AO is in direction of AB
        if (ab.dotProduct(ao) > 0) {
            Vector perp(-ab.y, ab.x);
            if (perp.dotProduct(ao) < 0) perp = Vector(ab.y, -ab.x);
            direction = perp;
        } else {
            // Remove B
            simplex.points = {a};
            direction = ao;
        }
        return false;

    } else if (count == 3) {
        Vector a = simplex.points[2];
        Vector b = simplex.points[1];
        Vector c = simplex.points[0];

        Vector ab = b - a;
        Vector ac = c - a;
        Vector ao = -a;

        Vector abPerp(ab.y, -ab.x);
        if (abPerp.dotProduct(ac) > 0) abPerp = Vector(-ab.y, ab.x);

        Vector acPerp(ac.y, -ac.x);
        if (acPerp.dotProduct(ab) > 0) acPerp = Vector(-ac.y, ac.x);

        // Check if origin is outside AB
        if (ab.dotProduct(ao) > 0 && abPerp.dotProduct(ao) > 0) {
            // Remove C
            simplex.points.erase(simplex.points.begin());
            direction = abPerp;
            return false;
        }

        // Check if origin is outside AC
        if (ac.dotProduct(ao) > 0 && acPerp.dotProduct(ao) > 0) {
            // Remove B
            simplex.points.erase(simplex.points.begin()+1);
            direction = acPerp;
            return false;
        }

        // Origin is inside simplex
        return true;
    } else {
        // Invalid simplex size
        assert(false && "Invalid simplex size in handleSimplex.");
        direction = Vector(1,0);
        return false;
    }
}

bool GJKIntersect(const ShapeData &A, const ShapeData &B, Simplex &simplex) {
    // Defensive checks on shape data
    if (A.isCircle) {
        assert(A.radius > 0 && "Circle radius must be > 0.");
    } else {
        assert(A.poly.vertices.size() >= 3 && "Polygon must have at least 3 vertices.");
    }

    if (B.isCircle) {
        assert(B.radius > 0 && "Circle radius must be > 0.");
    } else {
        assert(B.poly.vertices.size() >= 3 && "Polygon must have at least 3 vertices.");
    }

    Vector direction(1,0);
    simplex.points.clear();
    simplex.points.push_back(supportMinkowski(A,B,direction));

    if (simplex.points[0].dotProduct(direction) < 0) {
        // No collision
        return false;
    }

    direction = -simplex.points[0];

    int iterationCount = 0;
    const int MAX_GJK_ITER = 100;

    while (true) {
        iterationCount++;
        if (iterationCount > MAX_GJK_ITER) {
            std::cerr << "Warning: GJK exceeded max iterations. Assuming no collision." << std::endl;
            return false;
        }

        Vector newPoint = supportMinkowski(A,B,direction);
        double proj = newPoint.dotProduct(direction);
        if (proj < 0) {
            // No collision
            return false;
        }

        simplex.points.push_back(newPoint);

        if (handleSimplex(simplex, direction)) {
            return true; // Collision detected
        }
    }

    // Unreachable
    return false; 
}