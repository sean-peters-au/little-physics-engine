#ifndef EPA_HPP
#define EPA_HPP

#include <optional>
#include "gjk.hpp"

/** EPA result with normal and penetration depth */
struct EPAResult {
    Vector normal;
    double penetration;
};

std::optional<EPAResult> EPA(const ShapeData &A, const ShapeData &B, const Simplex &simplex);

#endif