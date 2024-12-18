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
  this->screenPos.y += 10;
}

void CoordinateSystem::moveDown() {
  this->screenPos.y -= 10;
}

void CoordinateSystem::moveLeft() {
  this->screenPos.x += 10;
}

void CoordinateSystem::moveRight() {
  this->screenPos.x -= 10;
}

Position CoordinateSystem::screenToUniverse(Position pos) {
  // Convert screen coordinates to meters
  return Position(
    SimulatorConstants::pixelsToMeters(pos.x - screenPos.x + SimulatorConstants::ScreenLength / 2),
    SimulatorConstants::pixelsToMeters(pos.y - screenPos.y + SimulatorConstants::ScreenLength / 2)
  );
}

Position CoordinateSystem::universeToScreen(Position pos) {
  // Convert meters to screen coordinates
  return Position(
    SimulatorConstants::metersToPixels(pos.x) - SimulatorConstants::ScreenLength / 2 + screenPos.x,
    SimulatorConstants::metersToPixels(pos.y) - SimulatorConstants::ScreenLength / 2 + screenPos.y
  );
}
