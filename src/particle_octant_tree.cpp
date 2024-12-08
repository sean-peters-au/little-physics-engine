#include "particle_octant_tree.h"
#include "particle.h"
#include "vector_math.h"
#include "simulator_constants.h"

#include <cmath>
#include <vector>
#include <iostream>

ParticleOctantTree::ParticleOctantTree() {}

ParticleOctantTree::~ParticleOctantTree() {
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      delete(this->children[i][j]);
    }
  }
}

ParticleOctantTree::ParticleOctantTree(Position pos, double len, Particle* particle) {
  this->children = std::vector<std::vector<ParticleOctantTree*>> (
    2, 
    std::vector<ParticleOctantTree*>(2)
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

bool ParticleOctantTree::contains(Particle* particle) {
  return this->pos.x - particle->radius < particle->pos.x &&
    particle->pos.x < this->pos.x + this->len + particle->radius &&
    this->pos.y - particle->radius < particle->pos.y &&
    particle->pos.y < this->pos.y + this->len + particle->radius;
}

bool ParticleOctantTree::performCollisionEffectsOn(Particle* particle) {
  if (this->particle == NULL) {
    // non-leaf
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 2; ++j) {
        if(this->children[i][j] != NULL
          && this->children[i][j]->contains(particle)
          && this->children[i][j]->performCollisionEffectsOn(particle)) {
            return true;
        }
      }
    }
  } else {
    return this->particle->collide(particle);
  }

  return false;
}

void ParticleOctantTree::addParticleBelow(Particle* particle) {
  if (particle == NULL) {
    std::cout << "null particle added below" << std::endl;
  }
	double childLen = this->len / 2;
  //std::cout << this->pos.x << " " << this->pos.y << " " << childLen << std::endl;
  //std::cout << particle->pos.x << " " << particle->pos.y << std::endl;

	int childCol = (int)((particle->pos.x - this->pos.x) / childLen);
	int childRow = (int)((particle->pos.y - this->pos.y) / childLen);
  if (isnan(childCol) || isnan(childRow)) {
    std::cout << "nan col or row" << std::endl;
  }
	Position childPos = Position(
		childCol * childLen + this->pos.x, 
		childRow * childLen + this->pos.y
	);

  if (childCol < 0 || childCol > 1 ||
      childRow < 0 || childRow > 1) {
    // left universe
    std::cout << "bad particle: " << childCol << ":" << childRow << "\t(" << particle->pos.x << "," << particle->pos.y << ")\t(" << this->pos.x << "," << this->pos.y << ")" << std::endl;
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
    std::cout << "null particle added" << std::endl;
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

  if (this->particle == other) {
    // leaf (ignore self)
  } else if (this->len / this->centre.dist(other->pos) < SimulatorConstants::BarnesHutRatio) {
    // 'distant' particles can be approximated
    acceleration = this->approxGravitationalAccelerationOn(other);
  } else if (this->particle == NULL) {
    // non-leaf
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 2; ++j) {
        if (this->children[i][j] != NULL) {
          Vector v = this->children[i][j]->netGravitationalAccelerationOn(other);
          acceleration.add(v);
        }
      }
    }
  } else {
    acceleration = this->particle->gravitationalAccelerationOn(other);
  }

  return acceleration;
}

void ParticleOctantTree::updateDensities(double densityAbove) {
  // p = m/V
  double densityHere = this->mass / (this->len * this->len);
  if (densityHere < densityAbove) {
    densityHere = densityAbove;
  }
  
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      if (this->children[i][j] == NULL) {
        continue;
      } else if (this->children[i][j]->particle == NULL) {
        this->children[i][j]->updateDensities(densityHere);
      } else {
        this->children[i][j]->particle->density = densityHere;
      }
    }
  }
}
