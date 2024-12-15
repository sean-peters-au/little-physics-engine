#ifndef COMPONENTS_BASIC_HPP
#define COMPONENTS_BASIC_HPP

#include <cstdint>

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

    struct Position {
        double x;
        double y;
    };

    struct Velocity {
        double x;
        double y;
    };

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

} // namespace Components

#endif