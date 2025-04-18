/**
 * @fileoverview simulator.hpp
 * @brief Main simulator class managing an ECS registry and scenario lifecycle.
 */

#pragma once

#include <entt/entt.hpp>
#include <memory>
#include <vector>

#include "scenarios/i_scenario.hpp"
#include "systems/i_system.hpp"

/**
 * @class ECSSimulator
 * @brief Maintains ECS state, scenario data, and performs physics updates.
 */
class ECSSimulator {
 public:
  // Delete copy/move constructors and assignment operators
  ECSSimulator(const ECSSimulator&) = delete;
  ECSSimulator& operator=(const ECSSimulator&) = delete;
  ECSSimulator(ECSSimulator&&) = delete;
  ECSSimulator& operator=(ECSSimulator&&) = delete;

  /** @brief Get the singleton instance. */
  static ECSSimulator& getInstance();

  /**
   * @brief Loads a new scenario object that can create ECS entities.
   * @param scenario A unique pointer to the scenario object.
   */
  void loadScenario(std::unique_ptr<IScenario> scenario);

  /**
   * @brief Applies a SharedSystemConfig to the simulator's internal state.
   * @param cfg The scenario configuration with relevant parameters.
   */
  void applyConfig(const ScenarioSystemConfig& cfg);

  /**
   * @brief Clears and re-initializes the ECS registry using the loaded scenario.
   *
   * This removes existing entities, then calls createEntities(...) on the
   * loaded scenario, and finally performs any post-initialization steps.
   */
  void reset();

  /**
   * @brief Perform additional initialization steps.
   */
  void init();

  /**
   * @brief Advances the ECS systems by one tick.
   */
  void tick();

  /**
   * @brief Provides mutable access to the ECS registry.
   * @return Reference to the entt::registry.
   */
  entt::registry& getRegistry();

  /**
   * @brief Provides read-only access to the ECS registry.
   * @return Const reference to the entt::registry.
   */
  const entt::registry& getRegistry() const;

  /**
   * @brief Returns a reference to the currently loaded scenario.
   * @return IScenario reference if a scenario is loaded.
   * @note This call requires that a scenario has been loaded.
   */
  IScenario& getCurrentScenario() const;

 private:
  // Make constructor private
  ECSSimulator();
  // Destructor can remain default or be private if needed, public is fine too
  ~ECSSimulator();

  entt::registry registry;
  std::unique_ptr<IScenario> scenarioPtr;
  std::vector<std::unique_ptr<Systems::ISystem>> systems;
  ScenarioSystemConfig currentConfig;

  /**
   * @brief Create all system instances according to current config
   */
  void createSystems();
};