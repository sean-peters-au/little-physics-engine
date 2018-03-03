#ifndef PARTICLE_SIMULATOR_H
#define PARTICLE_SIMULATOR_H

#include "particle.h"

#include <vector>

class ParticleSimulator
{
    public:
		std::vector<Particle> particles;

    void generateParticles();

		void init();

		void tick();
};

#endif
