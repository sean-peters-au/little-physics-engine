#include "generators.h"
#include "simulator_constants.h"
#include "particle.h"
#include "vector_math.h"
#include "coordinates.h"

#include <random>
#include <time.h>
#include <vector>
#include <iostream>

class UniformScreenParticleGenerator : public ParticleGenerator 
{
	public:
	unsigned int padding;
  std::default_random_engine re{static_cast<unsigned int>(time(0))};

	std::uniform_real_distribution<double> positions;
	std::normal_distribution<> velocities;
	std::normal_distribution<> masses;
	CoordinateSystem* coordSystem;

	UniformScreenParticleGenerator(CoordinateSystem* coordSystem, unsigned int padding) {
    this->padding = padding;
    this->coordSystem = coordSystem; 
	  positions = std::uniform_real_distribution<double> (
      padding, 
      SimulatorConstants::ScreenLength - padding
    );
		velocities = std::normal_distribution<> (
			SimulatorConstants::ParticleVelocityMean, 
			SimulatorConstants::ParticleVelocityStdDev
		);
		masses = std::normal_distribution<> (
			SimulatorConstants::ParticleMassMean, 
			SimulatorConstants::ParticleMassStdDev
		);
	}

	Particle* generateParticle() {
		return new Particle(
    coordSystem->screenToUniverse(Position(positions(re), positions(re))),
			masses(re),
			Vector(velocities(re), velocities(re))
		);
	}

	std::vector<Particle*> generateParticles() {
		std::vector<Particle*> particles;
    for (int i = 0; i < SimulatorConstants::ParticleCount; ++i) {
      particles.push_back(generateParticle());
    }
		return particles;
	}
};

ParticleGenerator* 
UniformScreenParticleGeneratorFactory::buildGenerator(CoordinateSystem* coordSystem) {
	// XXX magic number for the padding, but I'm starting to feel like I'm back at work 
  // with all software engineering going on, so fuck it...
  return new UniformScreenParticleGenerator(coordSystem, SimulatorConstants::ParticleSpawnPadding);
}

class RotatingParticleGenerator : public ParticleGenerator 
{
	public:
	unsigned int padding;
  std::default_random_engine re;

	std::uniform_real_distribution<double> positions;
	std::uniform_real_distribution<double> velocities;
	std::uniform_real_distribution<double> colors;
	std::normal_distribution<> masses;
  Position centre;

  CoordinateSystem* coordSystem;

	RotatingParticleGenerator(CoordinateSystem* coordSystem, unsigned int padding) {
    re.seed(time(0));
    this->padding = padding;
    this->coordSystem = coordSystem;
    this->centre = coordSystem->screenToUniverse(Position(
      SimulatorConstants::ScreenLength / 2,
      SimulatorConstants::ScreenLength / 2));

	  this->positions = std::uniform_real_distribution<double> (
      padding, 
      SimulatorConstants::ScreenLength - padding
    );
		this->velocities = std::uniform_real_distribution<double> (
			SimulatorConstants::ParticleVelocityMin,
			SimulatorConstants::ParticleVelocityMax
		);
		this->colors = std::uniform_real_distribution<double> (
      0,
			255
		);
		this->masses = std::normal_distribution<> (
			SimulatorConstants::ParticleMassMean, 
			SimulatorConstants::ParticleMassStdDev
		);
	}

	Particle* generateParticle() {
    Position pos = coordSystem->screenToUniverse(Position(positions(re), positions(re)));
    Vector v = Vector(pos - centre);
    v.rotate();
    v.scale(velocities(re));
		Particle* p = new Particle(pos, masses(re), v);
    p->r = (unsigned char)colors(re);
    p->g = (unsigned char)colors(re);
    p->b = (unsigned char)colors(re);
    p->a = (unsigned char)colors(re);
    return p;
	}

	std::vector<Particle*> generateParticles() {
		std::vector<Particle*> particles;
    for (int i = 0; i < SimulatorConstants::ParticleCount; ++i) {
      particles.push_back(generateParticle());
    }
		return particles;
	}
};

ParticleGenerator* 
RotatingParticleGeneratorFactory::buildGenerator(CoordinateSystem* coordSystem) {
	// XXX magic number for the padding, but I'm starting to feel like I'm back at work 
  // with all software engineering going on, so fuck it...
  return new RotatingParticleGenerator(coordSystem, SimulatorConstants::ParticleSpawnPadding);
}
