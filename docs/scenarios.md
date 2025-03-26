# Scenario Abstraction Overview

The scenario abstraction is designed to provide a unified interface for defining and executing simulation scenarios.

## Intent and Purpose

The abstraction of a scenario serves several key purposes:

- **Unified Scenario Definition:**  
  All scenarios must implement the `IScenario` interface. This ensures that every simulation scenario follows a consistent design by packaging both the simulation setup (configuration) and the entity construction in one unit.

- **Encapsulation of Simulation Behavior:**  
  The `IScenario` interface requires each scenario to specify:
  - A configuration phase via `getConfig()`, which returns a `ScenarioSystemConfig` structure defining simulation parameters (e.g., universe scale, physical constants, active systems).
  - An entity construction phase via `createEntities()`, which builds the simulation world using an ECS registry.

- **Modularity and Extensibility:**  
  By separating the “what” (simulation configuration) from the “how” (entity creation), the abstraction allows new scenarios to be developed by simply implementing the interface. This design promotes clean integration and makes it easier for tools, including code generators and LLMs, to produce new scenarios using the provided abstractions.

## Technical Design

The scenario abstraction is spread across multiple components:

- **`IScenario` (in `i_scenario.hpp`):**  
  This abstract interface defines the contract for a simulation scenario with two required methods:

  \[
  \texttt{ScenarioSystemConfig getConfig() const;}
  \]

  \[
  \texttt{void createEntities(entt::registry\& registry) const;}
  \]

  Every scenario must implement these methods. The configuration phase gathers all the simulation constants, while the entity creation phase populates the simulation with the necessary entities.

- **`ScenarioSystemConfig` (in `scenario_config.hpp` and `scenario_config.cpp`):**  
  Although an essential part of a scenario, `ScenarioSystemConfig` is a subsystem used by the scenario to hold specific simulation parameters such as universe size, physical properties, and the active systems. A helper function (`applyScenarioSystemConfig`) applies these configuration settings globally.

- **Illustrative Examples:**  
  Implementations like `SimpleFluidScenario` and `RandomPolygonsScenario` serve as illustrative examples. They show how to concretely implement a scenario by providing customized configuration and entity creation logic. (Note: the specifics of these examples are not to be documented in detail—the focus here is on the abstraction itself.)

## How to Use the Scenario Abstraction

Follow these steps to integrate a new scenario into the simulation:

1. **Implement a New Scenario:**

   Create a new class deriving from `IScenario` and implement both `getConfig()` and `createEntities()`. For example:

   ```cpp:src/scenarios/my_scenario.cpp
   #include "core/i_scenario.hpp"
   #include "core/scenario_config.hpp"

   class MyScenario final : public IScenario {
   public:
       ScenarioSystemConfig getConfig() const override {
           ScenarioSystemConfig cfg;
           // Define simulation parameters:
           cfg.MetersPerPixel = 1e-2;
           cfg.UniverseSizeMeters = 1000.0 * cfg.MetersPerPixel;
           cfg.SecondsPerTick = 1.0 / 60.0;  // 60 steps per second
           cfg.TimeAcceleration = 1.0;
           // Additional simulation parameters set here...
           cfg.activeSystems = {
               Systems::SystemType::BOUNDARY,
               Systems::SystemType::COLLISION,
               // Other systems as needed...
           };
           return cfg;
       }

       void createEntities(entt::registry &registry) const override {
           // Create simulation entities here.
           // For example, you could create boundaries, particles, etc.
       }
   };
   ```

2. **Apply the Configuration Globally:**

   Once you have an instance of your scenario, retrieve its configuration and apply it to set up the global simulation constants using the helper function. For example:

   ```cpp:src/some_file.cpp
   // 'scenario' is an instance of a class implementing IScenario,
   // and 'registry' is your ECS registry.
   ScenarioSystemConfig cfg = scenario.getConfig();
   applyScenarioSystemConfig(cfg);
   ```

   This call establishes the simulation settings (like universe size, physics properties, etc.) based on your scenario’s configuration.

3. **Create Entities:**

   Finally, create the required entities by invoking the entity creation method:

   ```cpp:src/some_file.cpp
   scenario.createEntities(registry);
   ```

   This step assembles the simulation world by creating entities within the ECS registry according to the logic defined in the scenario.

## Summary

- The **central abstraction** is encapsulated in the `IScenario` interface, which standardizes both simulation configuration and entity creation.
- `ScenarioSystemConfig` is an important internal subsystem of a scenario, used to define the simulation constants but is integrated into the overall scenario design.
- Implementing a new scenario involves:
  - Deriving from `IScenario`
  - Defining the configuration in `getConfig()`
  - Building the simulation's entity set in `createEntities()`
- Illustrative examples like `SimpleFluidScenario` and `RandomPolygonsScenario` demonstrate concrete implementations, serving as guidance rather than subjects for detailed documentation.

This abstraction approach provides a clean separation of concerns, enabling flexible, modular development of simulation scenarios while allowing tools and code generators to rapidly produce new scenario implementations.