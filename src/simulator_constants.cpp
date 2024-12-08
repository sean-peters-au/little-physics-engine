#include "simulator_constants.h"

namespace SimulatorConstants {
	extern const double Pi = 3.141592654;
	extern const double GravitationalConst = 6.674 * 1e-11;
  extern const double GasConst = 8.3144598;
  extern const double DragCoeff = 3e-8;
  
  // Drag = Coeff * density * v^2 / 2 * A

	extern const double GravitationalSoftener = 1e11;
  // gas clouds aren't going to bounce cleanly off each other
  // XXX: I could perhaps just implement some sort of "drag" and increase softening for a 
  // more realistic clumping
  extern const double CollisionCoeffRestitution = 0.05; 

  extern const double CollisionDistance = 0;

	extern const double BarnesHutRatio = 0.9;

	extern const double TimeStep = 1; // seconds
	extern const double PixelStep = 1e9; // metres
	extern const double MassStep = 1e12; // kilograms 

	extern const double ParticleCount = 3000;
	extern const double ParticleMassMean = 1e26; // kilograms
	extern const double ParticleMassStdDev = 0; 
	extern const double ParticleDensity = 1e4; // pascals i.e 1e-5 atmos
  // used by uniform screen generator
	extern const double ParticleVelocityMean = 0; // metres per second
	extern const double ParticleVelocityStdDev = 5e6;
  // used by rotating generator
	extern const double ParticleVelocityMin = 0; // metres per second
	extern const double ParticleVelocityMax = 0; // metres per second
  // this value is used by screen particle generators to determine how far from the screen 
  // edge particles can spawn
	extern const unsigned int ParticleSpawnPadding = 295;

	extern const double UniverseLength = 1e6 * PixelStep; // metres
	extern const unsigned int ScreenLength = 600;
	extern const unsigned int StepsPerSecond = 1000;
	extern const unsigned int Threads = 1;
}
