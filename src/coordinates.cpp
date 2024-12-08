#include "vector_math.h"
#include "simulator_constants.h"
#include "coordinates.h"

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
  double universeLengthPixels = SimulatorConstants::UniverseLength / SimulatorConstants::PixelStep;
  return Position(
    (pos.x - screenPos.x + universeLengthPixels / 2) * SimulatorConstants::PixelStep,
    (pos.y - screenPos.y + universeLengthPixels / 2) * SimulatorConstants::PixelStep
  );
}

Position CoordinateSystem::universeToScreen(Position pos) {
  double universeLengthPixels = SimulatorConstants::UniverseLength / SimulatorConstants::PixelStep;
  return Position(
    pos.x / SimulatorConstants::PixelStep - universeLengthPixels / 2 + screenPos.x,
    pos.y / SimulatorConstants::PixelStep - universeLengthPixels / 2 + screenPos.y
  );
}
