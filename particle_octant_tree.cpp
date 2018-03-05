#include "particle_octant_tree.h"
#include "particle.h"
#include "vector_math.h"
#include "simulator_constants.h"

#include <vector>
#include <iostream>

ParticleOctantTree::ParticleOctantTree() {}

ParticleOctantTree::ParticleOctantTree(Position pos, double len, Particle* particle) {
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

std::vector<Particle*> ParticleOctantTree::getParticles() {
	std::vector<Particle*> particles;

	if (this->particle) { // leaf
		particles.push_back(this->particle);
	} else { // non-leaf
		for (std::vector<ParticleOctantTree*> col : this->children) {
      for (ParticleOctantTree* child : col) {
        if (child != NULL) {
          std::vector<Particle*> childsParticles = child->getParticles();
          particles.insert(particles.end(), childsParticles.begin(), childsParticles.end());
        }
      }
		}
	}

	return particles;
}

void ParticleOctantTree::addParticleBelow(Particle* particle) {
  if (particle == NULL) {
    //std::cout << "null particle added below" << std::endl;
  }
	double childLen = this->len / 4;
  //std::cout << this->pos.x << " " << this->pos.y << " " << childLen << std::endl;
  //std::cout << particle->pos.x << " " << particle->pos.y << std::endl;

	if (this->particle != NULL && childLen < SimulatorConstants::CollisionDistance) {
		// TODO implement some sort of collision handling here
	}

	int childCol = (int)((particle->pos.x - this->pos.x) / childLen);
	int childRow = (int)((particle->pos.y - this->pos.y) / childLen);
	Position childPos = Position(
		childCol * childLen + this->pos.x, 
		childRow * childLen + this->pos.y
	);

  if (childCol < 0 || childCol > SimulatorConstants::UniverseLength ||
      childRow < 0 || childRow > SimulatorConstants::UniverseLength) {
    // left universe
    //std::cout << "a particle has left the universe" << std::endl;
  } else if (this->children[childCol][childRow] == NULL) {
    // new sub octant tree
		this->children[childCol][childRow] = 
			new ParticleOctantTree(childPos, childLen, particle);
	} else {
    // old sub octant tree
		this->children[childCol][childRow]->addParticle(particle);
	}
}

void ParticleOctantTree::addParticle(Particle* particle) {
  if (particle == NULL) {
    //std::cout << "null particle added" << std::endl;
  }
	if (this->particle != NULL) {
    //std::cout << "splitting" << std::endl;
		this->addParticleBelow(this->particle);
    this->particle = NULL;
	}

	this->addParticleBelow(particle);
	this->mass += particle->mass;
}

Vector ParticleOctantTree::approxGravitationalAccelerationOn(Particle* other) {
  Particle com = Particle(this->centre, this->mass, Vector(0,0));
  return com.gravitationalAccelerationOn(other);
}

Vector ParticleOctantTree::netGravitationalAccelerationOn(Particle* other) {
  Vector acceleration(0, 0);

  if (this->len / this->centre.dist(other->pos) < SimulatorConstants::BarnesHutRatio) {
    // 'distant' particles can be approximated
    acceleration = this->approxGravitationalAccelerationOn(other);
  } else if (this->particle == NULL) {
    // non-leaf
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        if (this->children[i][j] != NULL) {
          Vector v = this->children[i][j]->netGravitationalAccelerationOn(other);
          acceleration.add(v);
        }
      }
    }
  } else if (this->particle != other) {
    // leaf (ignore self)
    acceleration = this->particle->gravitationalAccelerationOn(other);
  }

  return acceleration;
}
