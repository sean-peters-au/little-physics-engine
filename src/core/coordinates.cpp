/**
 * @file coordinates.cpp
 * @brief Implementation of coordinate conversion utilities
 */

#include "core/coordinates.hpp"

namespace Simulation {

Coordinates::Coordinates(const SharedSystemConfig& config, unsigned int screenSize)
    : screenSize(screenSize),
      universeSizeMeters(config.UniverseSizeMeters)
{
    // Calculate meters per pixel based on universe size and screen size
    metersPerPixel = universeSizeMeters / static_cast<double>(screenSize);
}

double Coordinates::pixelsToMeters(double pixels) const {
    return pixels * metersPerPixel;
}

double Coordinates::metersToPixels(double meters) const {
    return meters / metersPerPixel;
}

double Coordinates::normalizedToMeters(double normalized) const {
    return normalized * universeSizeMeters;
}

double Coordinates::metersToNormalized(double meters) const {
    return meters / universeSizeMeters;
}

void Coordinates::updateConfig(const SharedSystemConfig& config) {
    universeSizeMeters = config.UniverseSizeMeters;
    // Recalculate meters per pixel
    metersPerPixel = universeSizeMeters / static_cast<double>(screenSize);
}

} // namespace Simulation