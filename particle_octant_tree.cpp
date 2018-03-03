#include "particle_octant_tree.h"
#include "particle.h"
#include "vector_math.h"
#include "simulator_constants.h"

#include <vector>

ParticleOctantTree::ParticleOctantTree() {}

ParticleOctantTree::ParticleOctantTree(Position pos, double len, Particle particle) {
  this->children = std::vector<std::vector<ParticleOctantTree*>> (
    4, 
    std::vector<ParticleOctantTree*>(4)
  );
	this->pos = pos;
	this->len = len;
	this->particle = particle;
	this->centre = Position(pos.x + len / 2, pos.y + len / 2);
	this->mass = 0;
}

std::vector<Particle> ParticleOctantTree::getParticles() {
	std::vector<Particle> particles;

	if (this->particle.null) { // leaf
		particles.push_back(this->particle);
	} else { // non-leaf
		for (std::vector<ParticleOctantTree*> col : this->children) {
      for (ParticleOctantTree* child : col) {
        if (child != NULL) {
          std::vector<Particle> childsParticles = child->getParticles();
          particles.insert(particles.end(), childsParticles.begin(), childsParticles.end());
        }
      }
		}
	}

	return particles;
}

void ParticleOctantTree::addParticleBelow(Particle particle) {
	double childLen = this->len / 4;

	if (this->particle.null && childLen < SimulatorConstants::CollisionDistance) {
		// TODO implement some sort of collision handling here
	}

	int childCol = (int)((particle.pos.x - this->pos.x) / childLen + 0.5);
	int childRow = (int)((particle.pos.y - this->pos.y) / childLen + 0.5);
	Position childPos = Position(
		childCol * childLen + this->pos.x, 
		childRow * childLen + this->pos.y
	);

	if (this->children[childCol][childRow] == NULL) {
		this->children[childCol][childRow] = 
			new ParticleOctantTree(childPos, childLen, particle);
	} else {
		this->children[childCol][childRow]->addParticle(particle);
	}
}

void ParticleOctantTree::addParticle(Particle particle) {
	if (!this->particle.null) {
		this->addParticleBelow(this->particle);
		this->particle.null = true;
	}

	this->addParticleBelow(particle);
	this->mass += particle.mass;
}

double ParticleOctantTree::approxGravitationalAccelerationOn(Particle other) {
	// TODO	
  return 0;
}

double ParticleOctantTree::netGravitationalAccelerationOn(Particle other) {
	// TODO	
  return 0;
}
