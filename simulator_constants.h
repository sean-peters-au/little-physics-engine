#ifndef SIMULATOR_CONSTANTS_H
#define SIMULATOR_CONSTANTS_H

//P = rho R T
// rho = density km.m^-2 (2D)

// R = 8.314 J.mol
// R = 8.3144598 (kPa.L)/(mol.K)
// n = number of mols
// p = pressure

// pV = nRT

// 1g H = 1mol

namespace SimulatorConstants {
	extern const double Pi;
	extern const double GravitationalConst;
  extern const double GasConst;
  extern const double DragCoeff;

	extern const double GravitationalSoftener;
  extern const double CollisionCoeffRestitution;

  extern const double CollisionDistance;

	extern const double BarnesHutRatio;

	extern const double TimeStep; // seconds
	extern const double PixelStep; // metres
	extern const double MassStep; // kilograms

	extern const double ParticleCount;
	extern const double ParticleMassMean; // kilograms
	extern const double ParticleMassStdDev; 
	extern const double ParticleDensity;
  // used by uniform screen generator
	extern const double ParticleVelocityMean; // metres per second
	extern const double ParticleVelocityStdDev;
  // used by rotating generator
	extern const double ParticleVelocityMin; // metres per second
	extern const double ParticleVelocityMax; // metres per second
	extern const unsigned int ParticleSpawnPadding;

	extern const double UniverseLength; 
	extern const unsigned int ScreenLength;
	extern const unsigned int StepsPerSecond;
	extern const unsigned int Threads;
}

#endif
