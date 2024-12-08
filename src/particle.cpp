#include "particle.h"
#include "vector_math.h"
#include "simulator_constants.h"

#include <math.h>
#include <iostream>

Particle::Particle() {}

Particle::Particle(Position pos, double mass, Vector velocity) {
  this->pos = pos;
  this->mass = mass;
  this->velocity = velocity;
  this->acceleration = Vector(0,0);
  this->moles = mass * 1e3;
  this->density = 0; // this will be calculated based on particles when sim starts
  this->volume = 3e20;
  this->radius = my_sqrt(this->volume / SimulatorConstants::Pi);
  this->temperature = 50;
}

void Particle::updateBoylesLaw() {
  // volume and moles
  // density calculated elsewhere
  // t = pv/nr  
  this->temperature = this->density * this->volume / this->moles * SimulatorConstants::GasConst;
  this->radius = my_sqrt(this->volume / SimulatorConstants::Pi);
}

Vector Particle::gravitationalAccelerationOn(Particle* other) {
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

  double dist = this->pos.dist(other->pos);
  double numerator = this->mass * SimulatorConstants::GravitationalConst;
  double denominator = my_sqrt(dist * dist + SimulatorConstants::GravitationalSoftener);
  denominator = denominator / (dist * dist) * denominator * denominator;

  double accelerationScalar = numerator / denominator;
  Vector accelerationVector = Vector(this->pos.x - other->pos.x, this->pos.y - other->pos.y);
  accelerationVector.scale(accelerationScalar);

  return accelerationVector;
}

bool Particle::collide(Particle* other)
{
  if (this >= other) {
    // avoids particles with themselves and ensure a collision between particles is only    
    // performed once
    return false;
  }

  // separation vector
  Vector d(other->pos - this->pos);

  // distance between circle centres, squared
  double distance_squared = d.dotProduct(d);

  // combined radius squared
  double radius = other->radius + this->radius;
  double radius_squared = radius * radius;

  // circles too far apart
  if(distance_squared > radius_squared) {
    return false;
  } else {
    //std::cout << "collision detected" << std::endl;
  }

  // collisionless drag implementation

  // XXX if we implement this drag effect linearly then we can do it for pairs of particles
  // XXX if the relationship isn't linear. Then you just want to calculate density based on 
  // the quad tree and apply some sort of density drag coeffecient

  // inelastic collision implementation

  // distance between circle centres
  double distance = my_sqrt(distance_squared);

  // normal of collision
  Vector ncoll = (d / distance);

  // penetration distance
  double dcoll = radius - d.length();

  // inverse masses (0 means, infinite mass, object is static).
  double ima = (this->mass > 0.0)? 1.0 / this->mass : 0.0;
  double imb = (other->mass > 0.0)? 1.0 / other->mass : 0.0;

  // separation vector
  Vector separation_vector = ncoll * (dcoll / (ima + imb));

  // separate the circles
  // XXX we're approximate gas clouds (which can intersect)
  this->pos -= separation_vector * ima;
  other->pos += separation_vector * imb;

  // combines velocity
  Vector vcoll = (other->velocity - this->velocity);

  // impact speed 
  double vn = vcoll.dotProduct(ncoll);

  // obejcts are moving away. dont reflect velocity
  if(vn > 0.0) 
    return true; // we did collide

  // coefficient of restitution in range [0, 1].
  const double cor = SimulatorConstants::CollisionCoeffRestitution; // air hockey -> high cor

  // collision impulse
  double j = vn / (ima + imb) * -(1.0 + cor);

  // collision impusle vector
  Vector impulse = ncoll * j;

  // change momentum of the circles
  this->velocity -= impulse * ima;
  other->velocity += impulse * imb;

  // collision reported
  return true;
}
