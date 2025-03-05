/**
 * @file random_polygons.cpp
 * @brief Implementation of a scenario with static boundary walls and random polygons/circles.
 *
 * This scenario places four walls around the simulation area and populates it
 * with circles or random polygons according to configurable probabilities. The
 * walls are treated as infinite-mass bodies, and particles have random masses,
 * sizes, and colors. Various friction parameters are specified for walls, floor,
 * and particles.
 */

#include <cmath>
#include <ctime>
#include <iostream>
#include <random>

#include "entities/entity_components.hpp"
#include "math/polygon.hpp"
#include "core/constants.hpp"
#include "systems/system_config.hpp"
#include "scenarios/random_polygons.hpp"

/**
 * @brief Creates a static "wall" entity with infinite mass, minimal thickness,
 *        placed asleep, and assigned a specific friction material.
 * @param registry The ECS registry where the entity is created.
 * @param cx Center X coordinate of the wall.
 * @param cy Center Y coordinate of the wall.
 * @param halfW Half the wall's width.
 * @param halfH Half the wall's height.
 * @param staticFriction The wall's static friction coefficient.
 * @param dynamicFriction The wall's dynamic friction coefficient.
 */
static void makeWall(entt::registry& registry,
                     double cx,
                     double cy,
                     double halfW,
                     double halfH,
                     double staticFriction,
                     double dynamicFriction) {
  auto wallEnt = registry.create();

  // Treat as infinite mass.
  double const mass = 1e30;

  registry.emplace<Components::Position>(wallEnt, cx, cy);
  registry.emplace<Components::Velocity>(wallEnt, 0.0, 0.0);
  registry.emplace<Components::Mass>(wallEnt, mass);
  registry.emplace<Components::ParticlePhase>(wallEnt, Components::Phase::Solid);
  registry.emplace<Components::Boundary>(wallEnt);

  auto& sleepC = registry.emplace<Components::Sleep>(wallEnt);
  sleepC.asleep = true;
  sleepC.sleepCounter = 9999999;

  registry.emplace<Components::Material>(wallEnt, staticFriction, dynamicFriction);

  PolygonShape poly;
  poly.type = Components::ShapeType::Polygon;
  poly.vertices.emplace_back(-halfW, -halfH);
  poly.vertices.emplace_back(-halfW, halfH);
  poly.vertices.emplace_back(halfW, halfH);
  poly.vertices.emplace_back(halfW, -halfH);

  registry.emplace<Components::Shape>(wallEnt, Components::ShapeType::Polygon, halfH);
  registry.emplace<PolygonShape>(wallEnt, poly);
  registry.emplace<Components::AngularPosition>(wallEnt, 0.0);
  registry.emplace<Components::Color>(wallEnt, 60, 60, 60);
}

SystemConfig RandomPolygonsScenario::getConfig() const {
  SystemConfig cfg;
  cfg.MetersPerPixel = 1e-2;
  cfg.UniverseSizeMeters =
      SimulatorConstants::ScreenLength * cfg.MetersPerPixel;
  cfg.SecondsPerTick = 1.0 / SimulatorConstants::StepsPerSecond;
  cfg.TimeAcceleration = 1.0;
  cfg.GridSize = 50;
  cfg.CellSizePixels =
      static_cast<double>(SimulatorConstants::ScreenLength) / cfg.GridSize;

  cfg.GravitationalSoftener = 0.0;
  cfg.CollisionCoeffRestitution = 0.2;
  cfg.DragCoeff = 0.0;
  cfg.ParticleDensity = 0.5;

  return cfg;
}

/**
 * @brief Populates the simulation with boundary walls and a set of randomly
 *        generated polygons or circles, according to scenario configuration.
 * @param registry The ECS registry where new entities are created.
 */
void RandomPolygonsScenario::createEntities(entt::registry& registry) const {
  // Obtain scenario configuration so we stay consistent with getConfig().
  SystemConfig cfg = getConfig();
  double const universeSizeM = cfg.UniverseSizeMeters;
  int const totalParticles = scenarioConfig.particleCount;

  std::default_random_engine generator{
      static_cast<unsigned int>(time(nullptr))};

  // Create four thin boundary walls around the edges.
  double const wallThickness = scenarioConfig.wallThickness;
  double const halfWall = wallThickness * 0.5;

  makeWall(registry, 0.0, universeSizeM * 0.5, halfWall, universeSizeM * 0.5,
           scenarioConfig.wallStaticFriction, scenarioConfig.wallDynamicFriction);
  makeWall(registry, universeSizeM, universeSizeM * 0.5, halfWall,
           universeSizeM * 0.5, scenarioConfig.wallStaticFriction, scenarioConfig.wallDynamicFriction);
  makeWall(registry, universeSizeM * 0.5, 0.0, universeSizeM * 0.5, halfWall,
           scenarioConfig.floorStaticFriction, scenarioConfig.floorDynamicFriction);
  makeWall(registry, universeSizeM * 0.5, universeSizeM, universeSizeM * 0.5,
           halfWall, scenarioConfig.wallStaticFriction, scenarioConfig.wallDynamicFriction);

  // Randomly distribute particles across the universe space as regular polygons or circles.
  std::uniform_real_distribution<> pos_dist(universeSizeM * 0.1,
                                          universeSizeM * 0.9);
  std::uniform_real_distribution<> vel_dist(-2.0, 2.0);
  std::uniform_real_distribution<> shape_dist(0.0, 1.0);
  std::uniform_real_distribution<> size_ratio_dist(0.0, 1.0);
  std::uniform_int_distribution<> color_dist(0, 255);

  // Mass distribution
  std::normal_distribution<> mass_dist(scenarioConfig.particleMassMean, scenarioConfig.particleMassStdDev);

  // For regular polygons, we need to decide how many sides.
  std::uniform_int_distribution<> sides_dist(3, 8);

  for (int i = 0; i < totalParticles; ++i) {
    // 1. Basic entity creation with position, velocity, mass, etc.
    auto e = registry.create();

    double const x = pos_dist(generator);
    double const y = pos_dist(generator);
    registry.emplace<Components::Position>(e, x, y);

    double velScale = scenarioConfig.initialVelocityFactor;
    registry.emplace<Components::Velocity>(e, vel_dist(generator) * velScale,
                                         vel_dist(generator) * velScale);

    double const mass = std::max(0.1, mass_dist(generator));
    registry.emplace<Components::Mass>(e, mass);
    registry.emplace<Components::ParticlePhase>(e, Components::Phase::Solid);
    registry.emplace<Components::Sleep>(e);

    // 2. Generate shape based on random distribution
    double const shape_type = shape_dist(generator);
    double size;

    // Decide if this is a small or large shape
    if (size_ratio_dist(generator) < scenarioConfig.smallShapeRatio) {
      // Small shape
      std::uniform_real_distribution<> smallDist(scenarioConfig.smallShapeMin, 
                                               scenarioConfig.smallShapeMax);
      size = smallDist(generator);
    } else {
      // Large shape
      std::uniform_real_distribution<> largeDist(scenarioConfig.largeShapeMin, 
                                               scenarioConfig.largeShapeMax);
      size = largeDist(generator);
    }

    // Apply friction coefficient for this particle
    registry.emplace<Components::Material>(
        e, scenarioConfig.particleStaticFriction, scenarioConfig.particleDynamicFriction);

    // 3. Shape creation
    if (shape_type < scenarioConfig.circlesFraction) {
      // Circle
      registry.emplace<Components::Shape>(e, Components::ShapeType::Circle, size);

      // Calculate moment of inertia for a circle: I = (1/2) * m * r²
      double I = 0.5 * mass * size * size;
      registry.emplace<Components::Inertia>(e, I);
      
      // Add CircleShape component
      registry.emplace<CircleShape>(e, size);
    } else if (shape_type < (scenarioConfig.circlesFraction + scenarioConfig.regularFraction)) {
      // Regular polygon
      int sides = sides_dist(generator);
      PolygonShape poly = buildRegularPolygon(sides, size);

      registry.emplace<Components::Shape>(e, Components::ShapeType::Polygon, size);
      registry.emplace<PolygonShape>(e, poly);

      // Calculate moment of inertia for the polygon
      double I = calculatePolygonInertia(poly.vertices, mass);
      registry.emplace<Components::Inertia>(e, I);
    } else {
      // Random polygon
      PolygonShape poly = buildRandomConvexPolygon(generator, size);

      registry.emplace<Components::Shape>(e, Components::ShapeType::Polygon, size);
      registry.emplace<PolygonShape>(e, poly);

      // Calculate moment of inertia for the polygon
      double I = calculatePolygonInertia(poly.vertices, mass);
      registry.emplace<Components::Inertia>(e, I);
    }

    // 4. Angular components
    registry.emplace<Components::AngularPosition>(e, 0.0);
    registry.emplace<Components::AngularVelocity>(e, vel_dist(generator) * 0.5);

    // 5. Random color
    registry.emplace<Components::Color>(e, color_dist(generator),
                                      color_dist(generator),
                                      color_dist(generator));
  }
}
