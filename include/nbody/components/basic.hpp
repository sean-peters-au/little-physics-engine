#ifndef COMPONENTS_BASIC_HPP
#define COMPONENTS_BASIC_HPP

#include <cstdint>
#include "nbody/math/vector_math.hpp" // for Position, Vector

namespace Components {

    enum class Phase {
        Solid,
        Liquid,
        Gas
    };

    enum class ShapeType {
        Circle,
        Square
    };

    // Use the Position and Vector classes from vector_math.hpp
    using Position = ::Position;
    using Velocity = ::Vector;

    struct Mass {
        double value;
    };

    struct Radius {
        double value; // For backward compatibility if needed
    };

    struct ParticlePhase {
        Phase phase;
    };

    struct Density {
        double value;
    };

    struct Temperature {
        double value;
    };

    // Shape Component
    // For circle: size = radius
    // For square: size = half side length
    struct Shape {
        ShapeType type;
        double size;
    };

    // Angular components
    struct AngularPosition {
        double angle; // radians
    };

    struct AngularVelocity {
        double omega; // radians per second
    };

    struct Inertia {
        double I; // moment of inertia
    };

    struct Color {
        uint8_t r, g, b;
        Color(uint8_t r = 255, uint8_t g = 255, uint8_t b = 255) 
            : r(r), g(g), b(b) {}
    };

    struct Sleep {
        int sleepCounter = 0;
        bool asleep = false;
    };

} // namespace Components

#endif