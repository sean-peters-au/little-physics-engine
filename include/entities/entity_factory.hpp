#pragma once

#include "entt/entt.hpp"
#include "entities/entity_components.hpp"

namespace Entities {

/**
 * Factory for creating common entity types in the simulation.
 * This class provides methods to create standardized entities
 * with appropriate components attached.
 */
class EntityFactory {
public:
    /**
     * Creates a basic particle entity with position, velocity, mass, and radius.
     *
     * @param registry The entity registry
     * @param position Initial position of the particle
     * @param velocity Initial velocity of the particle
     * @param mass Mass of the particle
     * @param radius Radius of the particle
     * @return The created entity
     */
    static entt::entity createBasicParticle(
        entt::registry& registry,
        const Components::Position& position,
        const Components::Velocity& velocity = Components::Velocity(),
        double mass = 1.0,
        double radius = 1.0
    );

    // Add more factory methods as needed
};

} // namespace Entities 