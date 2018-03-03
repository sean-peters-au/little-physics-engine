#include "generators.h"
#include "simulator_constants.h"
#include "particle.h"
#include "vector_math.h"
#include "coordinates.h"

#include <random>
#include <vector>

class UniformScreenParticleGenerator : public ParticleGenerator 
{
	public:
	unsigned int padding;
	std::default_random_engine re;

	std::uniform_real_distribution<double> positions;
	std::normal_distribution<> velocities;
	std::normal_distribution<> masses;

	UniformScreenParticleGenerator(unsigned int padding) : padding(padding) {
		// lower and upper bounds are obviously not a position, but this hack works as long as 
		// the screen is square.
		Position bounds = screenToUniverse(Position(0, SimulatorConstants::ScreenLength));

	  positions = std::uniform_real_distribution<double> (bounds.x, bounds.y);
		velocities = std::normal_distribution<> (
			SimulatorConstants::ParticleVelocityMean, 
			SimulatorConstants::ParticleVelocityStdDev
		);
		masses = std::normal_distribution<> (
			SimulatorConstants::ParticleMassMean, 
			SimulatorConstants::ParticleMassStdDev
		);
	}

	Particle generateParticle() {
		return Particle(screenToUniverse(Position(
				padding + positions(re) * (SimulatorConstants::ScreenLength - padding * 2),
				padding + positions(re) * (SimulatorConstants::ScreenLength - padding * 2))
			),
			masses(re),
			Vector(velocities(re), velocities(re))
		);
	}

	std::vector<Particle> generateParticles() {
		std::vector<Particle> particles;
		particles.push_back(generateParticle());
		return particles;
	}
};

ParticleGenerator* UniformScreenParticleGeneratorFactory::buildGenerator() {
	// XXX magic number for the padding, but I'm starting to feel like I'm back at work 
  // with all software engineering going on, so fuck it...
  return new UniformScreenParticleGenerator(50);
}
