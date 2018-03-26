#include "generators.h"
#include "factories.h"
#include "simulator_constants.h"
#include "simulator.h"
#include "particle.h"
#include "particle_octant_tree.h"
#include "vector_math.h"

#include <cmath>
#include <iostream>
#include <future>

ParticleSimulator::ParticleSimulator() {
  this->coordSystem = NULL;
}

ParticleSimulator::ParticleSimulator(CoordinateSystem* coordSystem) {
  this->coordSystem = coordSystem;
}

void ParticleSimulator::generateParticles() {
  ParticleGenerator::ParticleGenerator* generator = 
    SimulatorFactories::GeneratorFactory->buildGenerator(this->coordSystem);
  this->particles = generator->generateParticles();
}

void ParticleSimulator::init() {
  this->generateParticles();
}

ParticleOctantTree ParticleSimulator::createTree() {
  // construct octant tree
  ParticleOctantTree tree = 
    ParticleOctantTree(Position(0, 0), SimulatorConstants::UniverseLength, NULL);
  
  for (Particle* particle : this->particles) {
    tree.addParticle(particle);
  }

  return tree;
}

// this->algorithm is heavily based on the Barnes Hut algorithm for N Body simulations
void ParticleSimulator::tick() {
  // XXX Update position and velocity using Leapfrog Integration
  // XXX pretty sure this isn't leapfrog...
  double sumDensity = 0;
  double maxDensity = 0;
  double sumTemp = 0;
  double maxTemp = 0;
  for (Particle* particle : this->particles) {
    particle->pos += particle->velocity;
    sumDensity += particle->density;
    sumTemp += particle->temperature;
    if (particle->density > maxDensity) {
      maxDensity = particle->density;
    }
    if (particle->temperature > maxTemp) {
      maxTemp = particle->temperature;
    }
  }
  std::cout << "Density: Avg(" << sumDensity / SimulatorConstants::ParticleCount << ") \tMax(" << maxDensity << ")" << std::endl;
  std::cout << "Temp(K): Avg(" << sumTemp / SimulatorConstants::ParticleCount << ") \tMax(" << maxTemp << ")" << std::endl;

  ParticleOctantTree tree = this->createTree();

  // Update particle densities 
  tree.updateDensities(0);

  // Update particle volumes
  for (Particle* particle : this->particles) {
    particle->updateBoylesLaw();
  }

  // XXX gonna use drag not collisions
  //for (Particle* particle : this->particles) {
  //  tree.performCollisionEffectsOn(particle);
  //}
    
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
        (*particles)[i]->acceleration = acc * timestep;
      }
    });
  }
  for (int thread = 0; thread < SimulatorConstants::Threads; ++thread) {
    fut[thread].get();
  }

  // Calculate drag
  // Drag = Coeff * density * v^2 / 2 * A
  for (Particle* particle : this->particles) {
    double surfaceArea = my_sqrt(particle->volume / SimulatorConstants::Pi) * SimulatorConstants::Pi;
    Vector drag = Vector(
      SimulatorConstants::DragCoeff * particle->density * particle->velocity.x * particle->velocity.x / 2 * surfaceArea,
      SimulatorConstants::DragCoeff * particle->density * particle->velocity.y * particle->velocity.y / 2 * surfaceArea
    );
    // ensure drag applies against velocity of particle
    drag.x = (particle->velocity.x>0) ? -drag.x : drag.x;
    drag.y = (particle->velocity.y>0) ? -drag.y : drag.y;
    Vector dragAcc = drag / particle->mass * SimulatorConstants::TimeStep;
    if (dragAcc.length() > particle->acceleration.length() + particle->velocity.length()) {
      std::cout << "happened" << std::endl;
      particle->acceleration = Vector(0,0);
    } else {
      particle->acceleration += dragAcc;
    }
  }

  // Apply acceleration
  for (Particle* particle : this->particles) {
    particle->velocity += particle->acceleration;
    particle->acceleration = Vector(0,0);
  }
}
