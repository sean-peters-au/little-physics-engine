#include "particle.h"
#include "vector_math.h"
#include "simulator_constants.h"

#include <math.h>

Particle::Particle() {
  this->null = true;
}
Particle::Particle(Position pos, double mass, Vector velocity) {
  this->pos = pos;
  this->mass = mass;
  this->velocity = velocity;
  this->null = false;
}

Vector Particle::gravitationalAccelerationOn(Particle other) {
/* 
 * F =    Gm1m2r^2
 *    ---------------
 *    (sqrt(r^2+S))^3
 *
 * S is our softening constant. It handles the problem when bodies travel too closely,
 * and the equation begins to tend to infinity. A more correct solution would perhaps 
 * be to implement collision or some collision approximation. 
 *
 */

 double dist = this->pos.dist(other.pos);
 double numerator = SimulatorConstants::GravitationalConst;
 double denominator = sqrt(dist * dist + SimulatorConstants::GravitationalSoftener);
 denominator = denominator / (dist * dist) * denominator * denominator;

 double accelerationScalar = numerator / denominator;
 Vector accelerationVector = Vector(this->pos.x - other.pos.x, this->pos.y - other.pos.y);
 accelerationVector.scale(accelerationScalar);

 return accelerationVector;
}
