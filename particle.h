#ifndef PARTICLE_H
#define PARTICLE_H

#include "vector_math.h"

class Particle
{
    public:
		Position pos;
		double mass;
		Vector velocity;
    bool null;

		Particle();
		Particle(Position pos, double mass, Vector velocity);

		Vector gravitationalAccelerationOn(Particle other);
};

#endif
