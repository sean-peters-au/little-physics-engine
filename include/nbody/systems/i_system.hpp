/**
 * @file i_system.hpp
 * @brief Interface for all ECS systems in the nbody simulation
 */

#pragma once

#include <entt/entt.hpp>
#include "nbody/core/system_config.hpp"

namespace Systems {

/**
 * @class ISystem
 * @brief Base interface for all ECS systems
 * 
 * This interface ensures all systems have a common way to be updated and configured.
 * Systems can be dynamically enabled/disabled and configured at runtime.
 */
class ISystem {
public:
    /**
     * @brief Virtual destructor for proper cleanup of derived classes
     */
    virtual ~ISystem() = default;
    
    /**
     * @brief Updates the system for one simulation step
     * 
     * @param registry EnTT registry containing all entities and components
     */
    virtual void update(entt::registry& registry) = 0;
    
    /**
     * @brief Sets the system configuration
     * 
     * @param config System configuration parameters
     */
    virtual void setSystemConfig(const SystemConfig& config) = 0;
};

} // namespace Systems
