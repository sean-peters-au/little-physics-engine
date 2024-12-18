#ifndef GJK_HPP
#define GJK_HPP

#include <optional>
#include <vector>
#include "nbody/math/vector_math.hpp"
#include "nbody/math/polygon.hpp"

struct Simplex {
    std::vector<Vector> points;
};

bool GJKIntersect(const ShapeData &A, const ShapeData &B, Simplex &simplex);

#endif