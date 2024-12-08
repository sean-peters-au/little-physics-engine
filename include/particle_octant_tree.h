#ifndef PARTICLE_OCTANT_TREEH
#define PARTICLE_OCTANT_TREEH

#include "particle.h"
#include "vector_math.h"

#include <vector>

class ParticleOctantTree
{
  public:
    std::vector<std::vector<ParticleOctantTree*>> children;
    Position pos; // top left corner
    double len;
    Position centre;
    Particle* particle; // for leaf nodes
    double mass; // sum of mass within

    ParticleOctantTree();
    ~ParticleOctantTree();
    ParticleOctantTree(Position pos, double len, Particle* particle);

    std::vector<Particle*> getParticles();
    void addParticleBelow(Particle* particle);
    void addParticle(Particle* particle);

    bool contains(Particle* particle);
    bool performCollisionEffectsOn(Particle* particle);

    Vector approxGravitationalAccelerationOn(Particle* other);
    Vector netGravitationalAccelerationOn(Particle* other);

    void updateDensities(double densityAbove);
};

#endif
