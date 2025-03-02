/**
 * @file coordinates.hpp
 * @brief Coordinates conversion utilities between simulation and display spaces
 *
 * Handles conversion between:
 * - Meters (simulation space)
 * - Pixels (display space)
 * - Normalized coordinates (0-1 space)
 */
#pragma once

#include "nbody/core/constants.hpp"
#include "nbody/systems/i_system.hpp"

namespace Simulation {

/**
 * @class Coordinates
 * @brief Handles coordinate conversions between different spaces
 * 
 * This class provides methods to convert between simulation space (meters),
 * screen space (pixels), and normalized space (0-1).
 */
class Coordinates {
public:
    /**
     * @brief Construct a new Coordinates converter
     * 
     * @param config System configuration with universe size and other parameters
     * @param screenSize Screen size in pixels (default: from SimulatorConstants)
     */
    explicit Coordinates(const SystemConfig& config, 
                         unsigned int screenSize = SimulatorConstants::ScreenLength);
    
    /**
     * @brief Convert from pixels to meters
     * 
     * @param pixels Position in screen space
     * @return double Position in simulation space (meters)
     */
    double pixelsToMeters(double pixels) const;
    
    /**
     * @brief Convert from meters to pixels
     * 
     * @param meters Position in simulation space
     * @return double Position in screen space (pixels)
     */
    double metersToPixels(double meters) const;
    
    /**
     * @brief Convert from normalized (0-1) to meters
     * 
     * @param normalized Position in normalized space (0-1)
     * @return double Position in simulation space (meters)
     */
    double normalizedToMeters(double normalized) const;
    
    /**
     * @brief Convert from meters to normalized (0-1)
     * 
     * @param meters Position in simulation space
     * @return double Position in normalized space (0-1)
     */
    double metersToNormalized(double meters) const;
    
    /**
     * @brief Get the current meters per pixel ratio
     * 
     * @return double Meters per pixel
     */
    double getMetersPerPixel() const { return metersPerPixel; }
    
    /**
     * @brief Get the current pixels per meter ratio
     * 
     * @return double Pixels per meter
     */
    double getPixelsPerMeter() const { return 1.0 / metersPerPixel; }
    
    /**
     * @brief Update the configuration
     * 
     * @param config New system configuration
     */
    void updateConfig(const SystemConfig& config);

private:
    double metersPerPixel;       ///< Conversion ratio from pixels to meters
    double universeSizeMeters;   ///< Size of universe in meters
    unsigned int screenSize;     ///< Size of screen in pixels
};

} // namespace Simulation