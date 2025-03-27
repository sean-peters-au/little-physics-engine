#pragma once

#include <cstdint>
#include "math/vector_math.hpp"

namespace Components {

    enum class Phase {
        Solid,
        Liquid,
        Gas
    };

    enum class ShapeType {
        Circle,
        Square,
        Polygon
    };

    // Use the Position and Vector classes from vector_math.hpp
    using Position = ::Position;
    using Velocity = ::Vector;

    struct Mass {
        double value = 1.0; // Default mass of 1.0
        
        // Constructor for when we explicitly set mass
        explicit Mass(double v = 1.0) : value(v) {}
    };

    struct Radius {
        double value = 1.0; // Default radius of 1.0
        
        // Constructor for when we explicitly set radius
        explicit Radius(double v = 1.0) : value(v) {}
    };

    struct ParticlePhase {
        Phase phase = Phase::Solid; // Default to solid
        
        // Constructor for when we explicitly set phase
        explicit ParticlePhase(Phase p = Phase::Solid) : phase(p) {}
    };

    struct Density {
        double value = 1.0; // Default density of 1.0
        
        // Constructor for when we explicitly set density
        explicit Density(double v = 1.0) : value(v) {}
    };

    struct Temperature {
        double value = 298.15; // Default temperature (room temperature in K)
        
        // Constructor for when we explicitly set temperature
        explicit Temperature(double v = 298.15) : value(v) {}
    };

    // Shape Component
    // For circle: size = radius
    // For square: size = half side length
    struct Shape {
        ShapeType type = ShapeType::Circle; // Default to circle
        double size = 1.0; // Default size of 1.0
        
        // Constructor for when we explicitly set shape
        Shape(ShapeType t = ShapeType::Circle, double s = 1.0) : type(t), size(s) {}
    };

    // Angular components
    struct AngularPosition {
        double angle = 0.0; // Default angle (radians)
        
        // Constructor for when we explicitly set angle
        explicit AngularPosition(double a = 0.0) : angle(a) {}
    };

    struct AngularVelocity {
        double omega = 0.0; // Default angular velocity (radians per second)
        
        // Constructor for when we explicitly set angular velocity
        explicit AngularVelocity(double o = 0.0) : omega(o) {}
    };

    struct Inertia {
        double I = 1.0; // Default moment of inertia
        
        // Constructor for when we explicitly set moment of inertia
        explicit Inertia(double i = 1.0) : I(i) {}
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

    struct Material {
        double staticFriction = 0.5; // Default static friction
        double dynamicFriction = 0.3; // Default dynamic friction
        
        // Constructor for when we explicitly set friction
        Material(double sf = 0.5, double df = 0.3) : staticFriction(sf), dynamicFriction(df) {}
    };

    struct Boundary {
        bool isBoundary = false;
        
        // Constructor for when we explicitly set boundary
        explicit Boundary(bool b = false) : isBoundary(b) {}
    };

    // ---- SPH Components ----

    // Speed of sound (for isothermal or simple EOS)
    struct SpeedOfSound {
        double value = 1000.0;
        explicit SpeedOfSound(double v = 1000.0) : value(v) {}
    };

    // Temporary storage for SPH computations each frame
    struct SPHTemp {
        double density = 0.0;
        double pressure = 0.0;
        double acc_x = 0.0;
        double acc_y = 0.0;
    };

} // namespace Components 