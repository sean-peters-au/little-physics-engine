#include "vector_math.h"
#include "simulator_constants.h"

Position screenToUniverse(Position pos) {
  double universeLengthPixels = SimulatorConstants::UniverseLength / SimulatorConstants::PixelStep;
  return Position(
    (pos.x + universeLengthPixels / 2) * SimulatorConstants::PixelStep,
    (pos.y + universeLengthPixels / 2) * SimulatorConstants::PixelStep
  );
}

Position universeToScreen(Position pos) {
  double universeLengthPixels = SimulatorConstants::UniverseLength / SimulatorConstants::PixelStep;
  return Position(
    pos.x / SimulatorConstants::PixelStep - universeLengthPixels / 2,
    pos.y / SimulatorConstants::PixelStep - universeLengthPixels / 2
  );
}

