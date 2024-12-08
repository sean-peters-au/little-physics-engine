#ifndef COMPONENTS_H
#define COMPONENTS_H

#include "vector_math.h"

namespace Components {
    struct Position {
        double x, y;
        
        Position() : x(0), y(0) {}
        Position(double x, double y) : x(x), y(y) {}
        operator ::Position() const { return ::Position(x, y); }
    };

    struct Velocity {
        double x, y;
        
        Velocity() : x(0), y(0) {}
        Velocity(double x, double y) : x(x), y(y) {}
        operator Vector() const { return Vector(x, y); }
    };

    struct Mass {
        double value;
        explicit Mass(double v = 0.0) : value(v) {}
    };

    struct Temperature {
        double value;
        explicit Temperature(double v = 0.0) : value(v) {}
    };

    struct Density {
        double value;
        explicit Density(double v = 0.0) : value(v) {}
    };

    struct Volume {
        double value;
        explicit Volume(double v = 0.0) : value(v) {}
    };
}

#endif 