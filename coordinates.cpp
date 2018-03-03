#include "vector_math.h"
#include "simulator_constants.h"

Position screenToUniverse(Position pos) {
  return Position(
    pos.x - SimulatorConstants::UniverseLength / 2,
    pos.y - SimulatorConstants::UniverseLength / 2
  );
}

Position universeToScreen(Position pos) {
  return Position(
    pos.x + SimulatorConstants::UniverseLength / 2,
    pos.y + SimulatorConstants::UniverseLength / 2
  );
}
