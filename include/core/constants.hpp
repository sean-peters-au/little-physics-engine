#pragma once

#include <vector>
#include <string>

namespace SimulatorConstants {
    enum class SimulationType {
        KEPLERIAN_DISK,
        RANDOM_POLYGONS,
        SIMPLE_FLUID,
        FLUID_AND_POLYGONS,
        HOURGLASSES,
        PLANETARY_OCEAN,
        GALTON_BOARD
    };

    // Fundamental constants (do not change by scenario):
    extern const double Pi;
    extern const double RealG;
    extern const double Epsilon;

    // Display constants that never change:
    extern const unsigned int ScreenLength;
    extern const unsigned int StepsPerSecond;

    // Utility conversions (optionally keep here if you like):
    double pixelsToMeters(double pixels, double metersPerPixel);
    double metersToPixels(double meters, double metersPerPixel);

    // Scenario listing:
    std::vector<SimulationType> getAllScenarios();
    std::string getScenarioName(SimulationType scenario);
}