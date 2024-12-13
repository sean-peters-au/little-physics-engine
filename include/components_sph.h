#ifndef COMPONENTS_SPH_H
#define COMPONENTS_SPH_H

namespace Components {
    // Smoothing length for SPH kernels
    struct SmoothingLength {
        double value;
        explicit SmoothingLength(double v = 1.0) : value(v) {}
    };

    // Speed of sound (for isothermal or simple EOS)
    struct SpeedOfSound {
        double value;
        explicit SpeedOfSound(double v = 1000.0) : value(v) {}
    };

    // Temporary storage for SPH computations each frame
    struct SPHTemp {
        double density = 0.0;
        double pressure = 0.0;
        double acc_x = 0.0;
        double acc_y = 0.0;
    };
}

#endif