#pragma once

namespace Components {
    // Smoothing length for SPH kernels
    struct SmoothingLength {
        double value = 1.0;
        explicit SmoothingLength(double v = 1.0) : value(v) {}
    };

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
}