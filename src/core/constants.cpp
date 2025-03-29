#include "core/constants.hpp"
#include <cmath>
#include <vector>

namespace SimulatorConstants {

    const double Pi      = 3.141592654;
    const double RealG   = 6.674e-11;
    const double Epsilon = 1e-9;

    // Display
    const unsigned int ScreenLength   = 600;
    const unsigned int StepsPerSecond = 120;

    double pixelsToMeters(double pixels, double metersPerPixel) {
        return pixels * metersPerPixel;
    }

    double metersToPixels(double meters, double metersPerPixel) {
        return meters / metersPerPixel;
    }

    // List scenarios:
    std::vector<SimulationType> getAllScenarios() {
        return {
            SimulationType::KEPLERIAN_DISK,
            SimulationType::RANDOM_POLYGONS,
            SimulationType::SIMPLE_FLUID,
            SimulationType::FLUID_AND_POLYGONS,
            SimulationType::HOURGLASSES,
            SimulationType::PLANETARY_OCEAN,
            SimulationType::GALTON_BOARD
        };
    }

    std::string getScenarioName(SimulationType scenario) {
        switch (scenario) {
            case SimulationType::KEPLERIAN_DISK:   return "KEPLERIAN_DISK";
            case SimulationType::RANDOM_POLYGONS:  return "RANDOM_POLYGONS";
            case SimulationType::SIMPLE_FLUID:     return "SIMPLE_FLUID";
            case SimulationType::FLUID_AND_POLYGONS: return "FLUID_AND_POLYGONS";
            case SimulationType::HOURGLASSES:      return "HOURGLASSES";
            case SimulationType::PLANETARY_OCEAN:  return "PLANETARY_OCEAN";
            case SimulationType::GALTON_BOARD:     return "GALTON_BOARD";
            default: return "UNKNOWN";
        }
    }

} // namespace SimulatorConstants