#include "generators.h"
#include "factories.h"
#include "simulator_constants.h"
#include "simulator.h"
#include "particle.h"
#include "particle_octant_tree.h"

#include <iostream>
#include <future>

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
  ParticleOctantTree tree = 
    ParticleOctantTree(Position(0, 0), SimulatorConstants::UniverseLength, NULL);
  
  for (Particle* particle : this->particles) {
    tree.addParticle(particle);
  }

  // if the number of bodies ever changes (perhaps due to collision)
  // particles = tree.getParticles();
  
  // XXX Update position and velocity using Leapfrog Integration
  for (Particle* particle : this->particles) {
    particle->pos.add(particle->velocity); 
  }
    
  std::vector<std::future<void>> fut(SimulatorConstants::Threads);
  unsigned int size = this->particles.size();
  for (int thread = 0; thread < SimulatorConstants::Threads; ++thread) {
    fut[thread] = std::async(
      std::launch::async, 
      [thread, 
      particles = &this->particles, 
      &size, 
      threads = SimulatorConstants::Threads,
      &tree,
      timestep = SimulatorConstants::TimeStep]
    { 
      for (int i = thread * size / threads; 
          i < (thread + 1) * size / threads; 
          ++i) {
        Vector acc = tree.netGravitationalAccelerationOn((*particles)[i]); // m per s
        acc.multiply(timestep); // metres per time step
        (*particles)[i]->velocity.add(acc);
      }
    });
  }
  for (int thread = 0; thread < SimulatorConstants::Threads; ++thread) {
    fut[thread].get();
  }
}
