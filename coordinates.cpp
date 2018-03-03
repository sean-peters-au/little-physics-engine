#include "vector_math.h"
#include "simulator_constants.h"

Position screenToUniverse(Position pos) {
  return Position(
    (pos.x + SimulatorConstants::UniverseLength / 2) * SimulatorConstants::PixelStep,
    (pos.y + SimulatorConstants::UniverseLength / 2) * SimulatorConstants::PixelStep
  );
}

Position universeToScreen(Position pos) {
  return Position(
    pos.x / SimulatorConstants::PixelStep - SimulatorConstants::UniverseLength / 2,
    pos.y / SimulatorConstants::PixelStep - SimulatorConstants::UniverseLength / 2
  );
}

