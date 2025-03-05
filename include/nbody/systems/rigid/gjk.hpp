/**
 * @file gjk.hpp
 * @brief Implementation of the Gilbert-Johnson-Keerthi (GJK) collision detection algorithm
 *
 * The GJK algorithm is an iterative method used to determine whether two convex shapes intersect.
 * It works by iteratively building a simplex (a set of points) that attempts to enclose the origin
 * in the Minkowski difference of the two shapes. If the origin is enclosed, the shapes intersect.
 * 
 * This implementation supports:
 * - Collision detection between convex polygons
 * - Collision detection between circles
 * - Collision detection between mixed shapes (polygon-circle)
 */

#pragma once

#include <optional>
#include <vector>
#include "nbody/math/vector_math.hpp"
#include "nbody/math/polygon.hpp"

/**
 * @brief Represents a simplex used in the GJK algorithm
 * 
 * A simplex is a set of points (up to 3 in 2D) that represents the current
 * working set of the GJK algorithm as it attempts to enclose the origin.
 */
struct Simplex {
    std::vector<Vector> points;  ///< Points making up the simplex (1-3 points in 2D)
};

/**
 * @brief Determines if two shapes intersect using the GJK algorithm
 * 
 * @param A First shape data containing either polygon vertices or circle information
 * @param B Second shape data containing either polygon vertices or circle information
 * @param[out] simplex The final simplex if intersection is found, undefined if no intersection
 * @return true if shapes intersect, false otherwise
 * 
 * @note If intersection is found, the simplex will contain exactly 3 points that
 *       enclose the origin in the Minkowski difference space
 */
bool GJKIntersect(const ShapeData &A, const ShapeData &B, Simplex &simplex);