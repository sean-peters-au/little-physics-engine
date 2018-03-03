#include "factories.h"
#include "generators.h"

namespace SimulatorFactories {
	ParticleGeneratorFactory* GeneratorFactory = new UniformScreenParticleGeneratorFactory();
}
