#ifndef PARTICLE_SIMULATOR_H
#define PARTICLE_SIMULATOR_H

#include "particle.h"
#include "particle_octant_tree.h"
#include "coordinates.h"

#include <vector>

class ParticleSimulator
{
    public:
		std::vector<Particle*> particles;
    CoordinateSystem* coordSystem;

    ParticleSimulator();
    ParticleSimulator(CoordinateSystem* coordSystem);
    void generateParticles();

    ParticleOctantTree createTree();

		void init();

		void tick();
};

#endif
