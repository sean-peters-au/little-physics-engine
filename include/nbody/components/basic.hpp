#ifndef COMPONENTS_BASIC_HPP
#define COMPONENTS_BASIC_HPP

#include <cstdint>

namespace Components {

    enum class Phase {
        Solid,
        Liquid,
        Gas
    };

    // New shape types
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
        double value;
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

    // New Shape component
    // size = radius for circles, half-width/half-height for squares
    struct Shape {
        ShapeType type;
        double size; 
        // For Circle: size = radius
        // For Square: size = half side length
    };

} // namespace Components

#endif