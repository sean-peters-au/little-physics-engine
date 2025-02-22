# Configuration and Constants Overview

This document describes the current state of configuration management within the project. It captures the mix of approaches used to handle simulation constants and parameters, including both global values and per-scenario overrides, as well as the prevalence of "magic numbers" throughout the codebase.

## Global Constants in SimulatorConstants

The project uses a namespace, `SimulatorConstants`, declared in [constants.hpp](include/nbody/core/constants.hpp) and defined in [constants.cpp](src/core/constants.cpp), to hold a blend of immutable and mutable values:

- **Immutable Constants:**  
  Values such as `Pi`, `RealG`, and `Epsilon` are defined as constants and represent fundamental physical parameters.

- **Display Constants:**  
  Constants like `ScreenLength`, `StepsPerSecond`, and `Threads` govern the simulation’s display and processing behavior and remain unchanged during runtime.

- **Scenario-Specific Variables:**  
  Variables such as `UniverseSizeMeters`, `TimeAcceleration`, `MetersPerPixel`, `SecondsPerTick`, and others (e.g., `ParticleCount`, `ParticleMassMean`, etc.) are declared globally and mutate based on the simulation scenario. These values are intended to be configured at runtime.

- **Initialization:**  
  The function `initializeConstants(SimulationType type)` (located in [constants.cpp](src/core/constants.cpp)) resets or initializes these mutable variables to default "safe" values, depending on the selected simulation type.

## Per-Scenario Configuration via ScenarioConfig

Each simulation scenario uses a `ScenarioConfig` structure (defined in [scenario_config.hpp](include/nbody/core/scenario_config.hpp) and implemented in [scenario_config.cpp](src/core/scenario_config.cpp)) to encapsulate its intended configuration. This structure includes many of the same parameters as the global variables in `SimulatorConstants`.

- **Scenario Override:**  
  Each scenario populates a `ScenarioConfig` instance with values tailored to its needs. A helper function, `applyScenarioConfig(const ScenarioConfig &cfg)`, then propagates these values to the global `SimulatorConstants` variables. This ties scenario behavior directly to the simulation’s global state.

- **Hybrid Approach:**  
  While this mechanism exists for centralizing configuration, it represents only part of the picture. Some parts of the code rely on these global settings, while others use hard-coded values.

## The Prevalence of Magic Numbers

A significant challenge in the current configuration strategy is the widespread use of magic numbers. Examples include:

- **Scenario Implementations:**  
  Files like [random_polygons.cpp](src/scenarios/random_polygons.cpp) define numerous constants (e.g., `KSmallShapeMin`, `KLargeShapeMax`, `KParticleCount`, etc.) that directly influence simulation behavior without being routed through the centralized configuration.

- **Systems and Solvers:**  
  In [position_solver.cpp](src/systems/rigid_body_collision/position_solver.cpp), the rigid body collision solver uses several hard-coded parameters (such as `PosSolverIterations`, `BaumgarteFactor`, and `PenetrationSlop`) that are embedded within the system logic.

These magic numbers make it unclear which values are meant to be configurable and contribute to the overall complexity of managing simulation parameters.

## Challenges and Observations

- **Lack of a Unified Strategy:**  
  The project currently employs a hybrid approach using global mutable variables, scenario-specific overrides, and scattered hard-coded values. This fragmentation can lead to inconsistencies and difficult maintenance.

- **Flexibility vs. Fragility:**  
  While the mutable global state allows quick adjustments to the simulation parameters, it also introduces fragility. Changes in one part of the code might have unintended effects on another, and tracking down the source of a value can be challenging.

- **Documentation Gaps:**  
  Not all constants are well-documented. The dual approach—configuration via `ScenarioConfig` and direct use of magic numbers—can confuse both developers and automated tools trying to generate or verify configuration data.

## Future Directions

- **Centralize Configuration:**  
  Consider refactoring the project to consolidate all simulation parameters—especially the magic numbers—into a single, coherent configuration system. This could involve moving many of the hard-coded values into configuration files or interfaces.

- **Immutable vs. Mutable:**  
  Reevaluate which values truly need to be mutable and which should remain constant. A clearer separation between fixed fundamental constants and adjustable simulation parameters would enhance maintainability.

- **Improve Documentation:**  
  Strengthen documentation in both the code and external documents to clarify the purpose of each configuration parameter, making it easier for developers and code generation tools (including LLMs) to understand and modify the simulation setup.

## Conclusion

The current configuration strategy is both flexible and fragmented. Global variables in `SimulatorConstants` provide a central point for many simulation parameters, while each scenario influences these parameters through a dedicated `ScenarioConfig`. However, magic numbers and hard-coded values still permeate various parts of the codebase, from scenario implementations to collision solvers. This document aims to capture the "lay of the land" as it stands, providing a reference point before a more cohesive configuration management system is implemented.
