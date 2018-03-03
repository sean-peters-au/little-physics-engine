#include "simulator.h"
#include "particle.h"
#include "factories.h"
#include "particle_octant_tree.h"
#include "simulator_constants.h"

void ParticleSimulator::generateParticles() {
	ParticleGenerator::ParticleGenerator* generator = 
		SimulatorFactories::GeneratorFactory->buildGenerator();
	this->particles = generator->generateParticles();
}

void ParticleSimulator::init() {
	this->generateParticles();				
}

// this->algorithm is heavily based on the Barnes Hut algorithm for N Body simulations
void ParticleSimulator::tick() {
	// construct octant tree
	ParticleOctantTree::ParticleOctantTree tree = 
		ParticleOctantTree(Position(0, 0), SimulatorConstants::UniverseLength, Particle());
	
	for (Particle particle : this->particles) {
		tree.addParticle(particle);
	}

	// if the number of bodies ever changes (perhaps due to collision)
	// particles = tree.getParticles();
	
	// Update position and velocity using Leapfrog Integration
	for (Particle particle : this->particles) {
		// TODO	
	}
}
