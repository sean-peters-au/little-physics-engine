/**
 * @file random_polygons_scenario.cpp
 * @brief Implementation of a scenario with static boundary walls and random polygons/circles.
 *
 *        Changes:
 *        - Thinner walls (0.1 m).
 *        - Added a distinct color for walls.
 *        - Emphasized that walls remain "asleep" with infinite mass.
 */

#include <random>
#include <cmath>
#include <ctime>
#include <iostream>

#include "nbody/scenarios/random_polygons.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/math/polygon.hpp"

static constexpr double KCirclesFraction     = 0.0;
static constexpr double KRegularFraction     = 1.0;
static constexpr double KRandomPolyFraction  = 1.0 - KCirclesFraction - KRegularFraction;

static constexpr double KSmallShapeRatio = 0.90;
static constexpr double KSmallShapeMin = 0.1;  // Smaller minimum
static constexpr double KSmallShapeMax = 0.25;  // Smaller shapes range
static constexpr double KLargeShapeMin = 0.3;   // Larger shapes range
static constexpr double KLargeShapeMax = 0.5;  // Larger maximum

static constexpr int    KParticleCount = 100;

static constexpr double KFloorStaticFriction = 0.6;
static constexpr double KFloorDynamicFriction = 0.4;
static constexpr double KWallStaticFriction = 0.2;
static constexpr double KWallDynamicFriction = 0.1;
static constexpr double KParticleStaticFriction = 0.3;
static constexpr double KParticleDynamicFriction = 0.1;

// Helper: build a regular polygon
PolygonShape buildRegularPolygon(int sides, double sz)
{
    PolygonShape poly;
    poly.type = Components::ShapeType::Polygon;
    double const angleStep = 2.0 * M_PI / static_cast<double>(sides);
    for(int i = 0; i < sides; i++) {
        double const angle = i * angleStep;
        double const x = sz * std::cos(angle);
        double const y = -sz * std::sin(angle);
        poly.vertices.emplace_back(x, y);
    }
    return poly;
}

// Helper: build a random polygon
PolygonShape buildRandomPolygon(std::default_random_engine &gen, double sz)
{
    PolygonShape poly;
    poly.type = Components::ShapeType::Polygon;

    std::uniform_int_distribution<int> sideDist(3, 7);
    std::uniform_real_distribution<double> radiusDist(0.5 * sz, sz);

    int const sides = sideDist(gen);
    double const angleStep = 2.0 * M_PI / static_cast<double>(sides);

    double angle = 0.0;
    for(int i = 0; i < sides; i++) {
        double const r = radiusDist(gen);
        double const x = r * std::cos(angle);
        double const y = r * std::sin(angle);
        poly.vertices.emplace_back(x, y);
        angle += angleStep;
    }
    return poly;
}

double calculatePolygonInertia(const std::vector<Vector>& vertices, double mass) {
    double numerator = 0.0;
    double denominator = 0.0;
    
    int const n = vertices.size();
    for (int i = 0; i < n; i++) {
        int const j = (i + 1) % n;
        double const cross = vertices[i].cross(vertices[j]);
        numerator += cross * (vertices[i].dotProduct(vertices[i]) + 
                            vertices[i].dotProduct(vertices[j]) + 
                            vertices[j].dotProduct(vertices[j]));
        denominator += cross;
    }
    
    // For a polygon with uniform density
    return (mass * numerator) / (6.0 * denominator);
}

ScenarioConfig RandomPolygonsScenario::getConfig() const
{
    ScenarioConfig cfg;
    cfg.MetersPerPixel = 1e-2;
    cfg.UniverseSizeMeters = SimulatorConstants::ScreenLength * cfg.MetersPerPixel;
    cfg.SecondsPerTick = 1.0 / SimulatorConstants::StepsPerSecond;
    cfg.TimeAcceleration = 1.0;
    cfg.GridSize = 50;
    cfg.CellSizePixels = static_cast<double>(SimulatorConstants::ScreenLength) / cfg.GridSize;

    cfg.ParticleCount = KParticleCount;
    cfg.ParticleMassMean = 1.0;
    cfg.ParticleMassStdDev = 0.1;
    cfg.GravitationalSoftener = 0;
    cfg.CollisionCoeffRestitution = 0.2;
    cfg.DragCoeff = 0.0;
    cfg.ParticleDensity = 0.5;
    cfg.InitialVelocityFactor = 1.0;

    cfg.activeSystems = {
        Systems::SystemType::BASIC_GRAVITY,
        Systems::SystemType::COLLISION,
        Systems::SystemType::DAMPENING,
        Systems::SystemType::SLEEP,
        Systems::SystemType::ROTATION,
        Systems::SystemType::MOVEMENT,
    };

    return cfg;
}

/**
 * @brief Create a static "wall" entity with infinite mass, minimal thickness,
 *        set asleep, and given a distinct color. 
 */
static void makeWall(entt::registry &registry,
                     double cx, double cy,
                     double halfW, double halfH,
                     double staticFriction, double dynamicFriction)
{
    auto wallEnt = registry.create();

    // Infinite mass => 1e30
    double const mass = 1e30;

    registry.emplace<Components::Position>(wallEnt, cx, cy);
    registry.emplace<Components::Velocity>(wallEnt, 0.0, 0.0);
    registry.emplace<Components::Mass>(wallEnt, mass);

    // Mark it as a boundary + solid so systems skip it
    registry.emplace<Components::ParticlePhase>(wallEnt, Components::Phase::Solid);
    registry.emplace<Components::Boundary>(wallEnt);

    // Force it to be asleep
    auto &sleepC = registry.emplace<Components::Sleep>(wallEnt);
    sleepC.asleep = true;
    sleepC.sleepCounter = 9999999; // large

    // A friction or material if needed
    registry.emplace<Components::Material>(wallEnt, staticFriction, dynamicFriction);

    // Use a rectangle polygon with CCW winding
    PolygonShape poly;
    poly.type = Components::ShapeType::Polygon;
    // Start at bottom-left, go CCW
    poly.vertices.emplace_back(-halfW, -halfH);  // Bottom-left
    poly.vertices.emplace_back(-halfW,  halfH);  // Top-left
    poly.vertices.emplace_back( halfW,  halfH);  // Top-right
    poly.vertices.emplace_back( halfW, -halfH);  // Bottom-right

    registry.emplace<Components::Shape>(wallEnt, Components::ShapeType::Polygon, halfH);
    registry.emplace<PolygonShape>(wallEnt, poly);

    // Angular data (irrelevant for a static wall, but required)
    registry.emplace<Components::AngularPosition>(wallEnt, 0.0);

    // Give the walls a distinct color (e.g. dark gray)
    registry.emplace<Components::Color>(wallEnt, 60, 60, 60);
}

void RandomPolygonsScenario::createEntities(entt::registry &registry) const
{
    std::default_random_engine generator{static_cast<unsigned int>(time(nullptr))};

    double const universeSizeM = SimulatorConstants::UniverseSizeMeters;

    // Thin walls: 0.1m thick
    double const wallThickness = 0.1;
    double const halfWall = wallThickness * 0.5;

    // Left wall
    makeWall(registry, 0.0, universeSizeM*0.5, halfWall, universeSizeM*0.5, KWallStaticFriction, KWallDynamicFriction);
    // Right wall
    makeWall(registry, universeSizeM, universeSizeM*0.5, halfWall, universeSizeM*0.5, KWallStaticFriction, KWallDynamicFriction);
    // Top wall
    makeWall(registry, universeSizeM*0.5, 0.0, universeSizeM*0.5, halfWall, KWallStaticFriction, KWallDynamicFriction);
    // Bottom wall
    makeWall(registry, universeSizeM*0.5, universeSizeM, universeSizeM*0.5, halfWall, KFloorStaticFriction, KFloorDynamicFriction);

    // Now create circles/polygons as before
    int const totalParticles = SimulatorConstants::ParticleCount;
    int const circlesCount   = static_cast<int>(std::round(KCirclesFraction * totalParticles));
    int const regularCount   = static_cast<int>(std::round(KRegularFraction * totalParticles));
    int const randomCount    = totalParticles - circlesCount - regularCount;

    std::cerr << "Circles: " << circlesCount << ", Regular: " << regularCount << ", Random: " << randomCount << "\n";

    int const side = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(totalParticles))));
    double const spacing = universeSizeM / (side + 1);

    std::normal_distribution<> velocityDist(0.0, 1.0);
    std::normal_distribution<> massDist(SimulatorConstants::ParticleMassMean,
                                        SimulatorConstants::ParticleMassStdDev);
    std::uniform_real_distribution<double> smallSizeDist(KSmallShapeMin, KSmallShapeMax);
    std::uniform_real_distribution<double> largeSizeDist(KLargeShapeMin, KLargeShapeMax);
    std::uniform_real_distribution<double> sizeSelector(0.0, 1.0); // To choose between small and large
    std::uniform_int_distribution<> colorDist(100, 255);

    int created = 0;
    int cCount = 0;
    int rCount = 0;

    std::cerr << "Creating " << totalParticles << " particles\n";
    for (int i = 0; i < side && created < totalParticles; ++i) {
        for (int j = 0; j < side && created < totalParticles; ++j) {
            double xM = (i + 1) * spacing;
            double yM = (j + 1) * spacing;

            auto entity = registry.create();
            double const massVal = massDist(generator);

            registry.emplace<Components::Position>(entity, xM, yM);
            registry.emplace<Components::Velocity>(entity,
                velocityDist(generator), velocityDist(generator));
            registry.emplace<Components::Mass>(entity, massVal);
            registry.emplace<Components::ParticlePhase>(entity, Components::Phase::Solid);
            registry.emplace<Components::Sleep>(entity);
            // Lower friction
            registry.emplace<Components::Material>(entity, KParticleStaticFriction, KParticleDynamicFriction);

            double sz;
            if (sizeSelector(generator) < KSmallShapeRatio) {
                sz = smallSizeDist(generator);
            } else {
                sz = largeSizeDist(generator);
            }

            double i; 

            if (cCount < circlesCount) {
                // Circle - use disc formula
                registry.emplace<Components::Shape>(entity, Components::ShapeType::Circle, sz);
                registry.emplace<CircleShape>(entity, sz);
                i = 0.5 * massVal * (sz * sz);  // Correct for circles
                cCount++;
            }
            else if (rCount < regularCount) {
                // Regular polygon
                std::uniform_int_distribution<int> sidesDist(3, 6);
                int const sides = sidesDist(generator);

                PolygonShape const poly = buildRegularPolygon(sides, sz);
                registry.emplace<Components::Shape>(entity, Components::ShapeType::Polygon, sz);
                registry.emplace<PolygonShape>(entity, poly);

                // Calculate proper moment of inertia for the polygon
                i = calculatePolygonInertia(poly.vertices, massVal);
                rCount++;
            }
            else {
                // Random polygon
                PolygonShape const poly = buildRandomPolygon(generator, sz);
                registry.emplace<Components::Shape>(entity, Components::ShapeType::Polygon, sz);
                registry.emplace<PolygonShape>(entity, poly);

                // Calculate proper moment of inertia for the polygon
                i = calculatePolygonInertia(poly.vertices, massVal);
            }

            registry.emplace<Components::AngularPosition>(entity, 0.0);
            registry.emplace<Components::AngularVelocity>(entity, 0.0);
            registry.emplace<Components::Inertia>(entity, i);

            // Random color
            int rr = colorDist(generator);
            int gg = colorDist(generator);
            int bb = colorDist(generator);
            registry.emplace<Components::Color>(entity, rr, gg, bb);

            created++;
        }
    }

    std::cerr << "Created " << created << " particles\n";
}
