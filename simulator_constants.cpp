#include "simulator_constants.h"

namespace SimulatorConstants {
	extern const double GravitationalConst = 6.674 * 1e-11;
	extern const double GravitationalSoftener = 1;

  extern const double CollisionDistance = 0;

	extern const double BarnesHutRatio = 0.9;

	extern const double TimeStep = 86400 * 100; // seconds
	extern const double PixelStep = 1e12; // metres
	extern const double MassStep = 1e12; // kilograms

	extern const double ParticleCount = 8000;
	extern const double ParticleMassMean = 1e24; // kilograms
	extern const double ParticleMassStdDev = 0; 
	extern const double ParticleVelocityMean = 0; // metres per second
	extern const double ParticleVelocityStdDev = 5e6;

	extern const double UniverseLength = 10000 * PixelStep; // metres
	extern const unsigned int ScreenLength = 600;
	extern const unsigned int StepsPerSecond = 1000;
	extern const unsigned int Threads = 4;
}
