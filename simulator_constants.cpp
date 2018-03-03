#include "simulator_constants.h"

namespace SimulatorConstants {
	extern const double GravitationalConst = 6.674 * 1e-11;
	extern const double GravitationalSoftener = 1;

  extern const double CollisionDistance = 0;

	extern const double BarnesHutRatio = 0.5;

	extern const double TimeStep = 86400; // seconds
	extern const double PixelStep = 1e12; // metres
	extern const double MassStep = 1e12; // kilograms

	extern const double ParticleCount = 2;
	extern const double ParticleMassMean = 1e12; // kilograms
	extern const double ParticleMassStdDev = 0; 
	extern const double ParticleVelocityMean = 0; // metres per second
	extern const double ParticleVelocityStdDev = 5e5;

	extern const unsigned int UniverseLength = 10000; 
	extern const unsigned int ScreenLength = 600;
}
