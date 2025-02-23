/**
 * @file epa.hpp
 * @brief Implementation of the Expanding Polytope Algorithm (EPA) for collision resolution
 *
 * The EPA algorithm is used after GJK has detected a collision to determine the
 * penetration depth and collision normal. It works by expanding the final GJK
 * simplex into a polytope that better approximates the Minkowski difference
 * surface, allowing for precise contact information calculation.
 * 
 * This implementation:
 * - Takes the final simplex from GJK as input
 * - Iteratively expands the polytope until the closest edge to origin is found
 * - Provides penetration depth and collision normal for collision response
 */

#ifndef EPA_HPP
#define EPA_HPP

#include <optional>
#include "gjk.hpp"

/**
 * @brief Contains the collision resolution data computed by EPA
 * 
 * Stores the collision normal (direction of minimum penetration) and
 * the penetration depth (amount of overlap between shapes).
 */
struct EPAResult {
    Vector normal;     ///< Direction of minimum penetration (normalized)
    double penetration; ///< Depth of penetration between shapes
};

/**
 * @brief Calculates penetration depth and normal for intersecting shapes
 * 
 * @param A First shape data containing either polygon vertices or circle information
 * @param B Second shape data containing either polygon vertices or circle information
 * @param simplex The final simplex from GJK containing 3 points
 * @return std::optional<EPAResult> Contains penetration info if successful, nullopt if EPA fails
 * 
 * @note This function should only be called after GJK has confirmed an intersection
 *       and provided a valid simplex containing exactly 3 points
 */
std::optional<EPAResult> EPA(const ShapeData &A, const ShapeData &B, const Simplex &simplex);

#endif