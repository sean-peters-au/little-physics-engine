#include "factories.h"
#include "generators.h"

namespace SimulatorFactories {
	ParticleGeneratorFactory* GeneratorFactory = new RotatingParticleGeneratorFactory();
	//ParticleGeneratorFactory* GeneratorFactory = new UniformScreenParticleGeneratorFactory();
}
