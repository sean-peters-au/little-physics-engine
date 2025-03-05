/**
 * @file i_system.hpp
 * @brief Interface for all ECS systems in the nbody simulation
 */

#pragma once

#include <entt/entt.hpp>
#include "systems/system_config.hpp"

namespace Systems {

/**
 * @class ISystem
 * @brief Base interface for all ECS systems
 * 
 * This interface ensures all systems have a common way to be updated and configured.
 * Systems can be dynamically enabled/disabled and configured at runtime.
 */
class ISystem {
protected:
    SystemConfig sysConfig;  // Common configuration all systems have

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
    virtual void setSystemConfig(const SystemConfig& config) {
        sysConfig = config;
    }
    
    /**
     * @brief Gets the system configuration
     * 
     * @return Current system configuration
     */
    virtual const SystemConfig& getSystemConfig() const {
        return sysConfig;
    }
};

/**
 * @brief Template for system-specific configurations
 * 
 * This template can be used by derived systems that need additional
 * configuration beyond the basic SystemConfig.
 */
template<typename SpecificConfig>
class ConfigurableSystem : public ISystem {
protected:
    SpecificConfig specificConfig;
    
public:
    /**
     * @brief Sets the system-specific configuration
     * 
     * @param config System-specific configuration parameters
     */
    void setSpecificConfig(const SpecificConfig& config) {
        specificConfig = config;
    }
    
    /**
     * @brief Gets the system-specific configuration
     * 
     * @return Current system-specific configuration
     */
    const SpecificConfig& getSpecificConfig() const {
        return specificConfig;
    }
};

} // namespace Systems
