#ifndef PARTICLE_H
#define PARTICLE_H

#include "vector_math.h"

class Particle
{
    public:
		Position pos;
		Vector velocity;
		Vector acceleration;
		double mass;
    double temperature;
    double moles;
    double density;
    double volume;
    double radius;
    unsigned char r;
    unsigned char g;
    unsigned char b;
    unsigned char a;

		Particle();
		Particle(Position pos, double mass, Vector velocity);

		Vector gravitationalAccelerationOn(Particle* other);
    bool collide(Particle* other);
    void updateBoylesLaw();
};

#endif
