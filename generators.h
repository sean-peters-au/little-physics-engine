#ifndef GENERATORS_H
#define GENERATORS_H

#include "particle.h"
#include "coordinates.h"

#include <vector>

class ParticleGenerator {
	public:
		virtual std::vector<Particle*> generateParticles() = 0;
};

class ParticleGeneratorFactory {
	public:
		virtual ParticleGenerator* buildGenerator(CoordinateSystem* coordSystem) = 0;
};

class UniformScreenParticleGeneratorFactory : public ParticleGeneratorFactory {
	ParticleGenerator* buildGenerator(CoordinateSystem* coordSystem);
};

class RotatingParticleGeneratorFactory : public ParticleGeneratorFactory {
	ParticleGenerator* buildGenerator(CoordinateSystem* coordSystem);
};

#endif
