#ifndef GENERATORS_H
#define GENERATORS_H

#include "particle.h"

#include <vector>

class ParticleGenerator {
	public:
		virtual std::vector<Particle> generateParticles() = 0;
};

class ParticleGeneratorFactory {
	public:
		virtual ParticleGenerator* buildGenerator() = 0;
};

class UniformScreenParticleGeneratorFactory : public ParticleGeneratorFactory {
	ParticleGenerator* buildGenerator();
};

#endif
