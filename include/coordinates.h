#ifndef COORDINATES_H
#define COORDINATES_H

#include "vector_math.h"

class CoordinateSystem
{
  public:
  Position screenPos;

  CoordinateSystem();
  CoordinateSystem(Position screenPos);

  void moveUp();
  void moveDown();
  void moveLeft();
  void moveRight();

  Position screenToUniverse(Position pos);
  Position universeToScreen(Position pos);
};

#endif
