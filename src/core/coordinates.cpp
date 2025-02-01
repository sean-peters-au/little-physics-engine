/**
 * @file coordinates.cpp
 * @brief Implementation of coordinate system transformations
 */

#include "nbody/math/vector_math.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/core/coordinates.hpp"

CoordinateSystem::CoordinateSystem() {
    this->screenPos = Position(0,0);
}

CoordinateSystem::CoordinateSystem(Position screenPos) {
    this->screenPos = screenPos;
}

void CoordinateSystem::moveUp() {
    this->screenPos.y += 10;  // Pan up in screen space
}

void CoordinateSystem::moveDown() {
    this->screenPos.y -= 10;  // Pan down in screen space
}

void CoordinateSystem::moveLeft() {
    this->screenPos.x += 10;  // Pan left in screen space
}

void CoordinateSystem::moveRight() {
    this->screenPos.x -= 10;  // Pan right in screen space
}

Position CoordinateSystem::screenToUniverse(Position pos) const {
    // Convert screen coordinates to meters:
    // 1. Offset by screen pan position
    // 2. Center on screen
    // 3. Convert to meters using scale factor
    return {
        SimulatorConstants::pixelsToMeters(pos.x - screenPos.x + SimulatorConstants::ScreenLength / 2),
        SimulatorConstants::pixelsToMeters(pos.y - screenPos.y + SimulatorConstants::ScreenLength / 2)
    };
}

Position CoordinateSystem::universeToScreen(Position pos) const {
    // Convert universe coordinates to screen space:
    // 1. Convert from meters to pixels
    // 2. Center on screen
    // 3. Apply screen pan offset
    return {
        SimulatorConstants::metersToPixels(pos.x) - SimulatorConstants::ScreenLength / 2 + screenPos.x,
        SimulatorConstants::metersToPixels(pos.y) - SimulatorConstants::ScreenLength / 2 + screenPos.y
    };
}
