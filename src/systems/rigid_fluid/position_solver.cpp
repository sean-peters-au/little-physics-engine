/**
 * @file rigid_fluid.cpp
 * @brief Robust constraint-based solver for fluid-solid boundary conditions
 */

#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream> // For debug if needed

#include <entt/entt.hpp>

#include "nbody/core/profile.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/math/vector_math.hpp"
#include "nbody/math/polygon.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sph.hpp"
#include "nbody/components/sim.hpp"
#include "nbody/systems/rigid_fluid/position_solver.hpp"

namespace Systems {

// Simplified constraint system with minimal parameters
namespace FluidRigidConfig {
    constexpr double SafetyMargin = 0.05;     // Safety margin to push particles clear of boundaries
    constexpr double MinPushDistance = 0.01;   // Minimum push distance for numeric stability
}

// Simple data structures for processing
struct FluidParticle {
    entt::entity id;
    Components::Position* pos;
    double radius;           // Using smoothing length as an approximation for particle radius
    
    FluidParticle() : id(entt::null), pos(nullptr), radius(0) {}
};

struct RigidBody {
    entt::entity id;
    Components::Position pos;
    Components::Shape shape;
    Components::AngularPosition angPos;
    const PolygonShape* polygon;
    
    // AABB for broad phase
    double minX, minY, maxX, maxY;
};

// Single constraint between a fluid particle and rigid body
struct PositionConstraint {
    FluidParticle* particle;
    Vector normal;           // Normalized direction to push particle
    double depth;            // Penetration depth
};

// Collect all fluid particles
std::vector<FluidParticle> collectFluidParticles(entt::registry& registry) {
    std::vector<FluidParticle> particles;
    
    auto view = registry.view<Components::ParticlePhase, 
                            Components::Position,
                            Components::SmoothingLength>();
    
    for (auto entity : view) {
        const auto& phase = view.get<Components::ParticlePhase>(entity);
        if (phase.phase == Components::Phase::Liquid) {
            FluidParticle particle;
            particle.id = entity;
            particle.pos = &view.get<Components::Position>(entity);
            particle.radius = view.get<Components::SmoothingLength>(entity).value * 0.5; // Half smoothing length as radius
            particles.push_back(particle);
        }
    }
    
    return particles;
}

// Collect all rigid bodies
std::vector<RigidBody> collectRigidBodies(entt::registry& registry) {
    std::vector<RigidBody> bodies;
    
    auto view = registry.view<Components::ParticlePhase, 
                            Components::Position,
                            Components::Shape,
                            Components::AngularPosition>();
    
    for (auto entity : view) {
        const auto& phase = view.get<Components::ParticlePhase>(entity);
        if (phase.phase == Components::Phase::Solid) {
            RigidBody body;
            body.id = entity;
            body.pos = view.get<Components::Position>(entity);
            body.shape = view.get<Components::Shape>(entity);
            body.angPos = view.get<Components::AngularPosition>(entity);
            
            // Get polygon shape if applicable
            if (body.shape.type == Components::ShapeType::Polygon) {
                body.polygon = registry.try_get<PolygonShape>(entity);
            } else {
                body.polygon = nullptr;
            }
            
            // Calculate AABB
            double margin = body.shape.size * 1.2; // Add safety margin
            body.minX = body.pos.x - margin;
            body.minY = body.pos.y - margin;
            body.maxX = body.pos.x + margin;
            body.maxY = body.pos.y + margin;
            
            // Refine AABB for polygons
            if (body.polygon && body.shape.type == Components::ShapeType::Polygon) {
                body.minX = body.maxX = body.pos.x;
                body.minY = body.maxY = body.pos.y;
                
                for (const auto& v : body.polygon->vertices) {
                    Vector rotated = v.rotateByAngle(body.angPos.angle);
                    double vx = body.pos.x + rotated.x;
                    double vy = body.pos.y + rotated.y;
                    
                    body.minX = std::min(body.minX, vx);
                    body.minY = std::min(body.minY, vy);
                    body.maxX = std::max(body.maxX, vx);
                    body.maxY = std::max(body.maxY, vy);
                }
                
                // Add margin
                double safetyMargin = 0.05;
                body.minX -= safetyMargin;
                body.minY -= safetyMargin;
                body.maxX += safetyMargin;
                body.maxY += safetyMargin;
            }
            
            bodies.push_back(body);
        }
    }
    
    return bodies;
}

// Transform polygon to world space
std::vector<Vector> getWorldSpacePolygon(const PolygonShape& poly, 
                                        const Vector& position, 
                                        double angle) {
    std::vector<Vector> worldVerts;
    worldVerts.reserve(poly.vertices.size());
    
    for (const auto& v : poly.vertices) {
        Vector rotated = v.rotateByAngle(angle);
        worldVerts.push_back(Vector(position.x + rotated.x, position.y + rotated.y));
    }
    
    return worldVerts;
}

// Properly detect if a point is inside a polygon using ray casting
bool isPointInPolygon(const Vector& point, const std::vector<Vector>& polygon) {
    bool inside = false;
    int nVerts = polygon.size();
    
    for (int i = 0, j = nVerts - 1; i < nVerts; j = i++) {
        if (((polygon[i].y > point.y) != (polygon[j].y > point.y)) &&
            (point.x < (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / 
            (polygon[j].y - polygon[i].y) + polygon[i].x)) {
            inside = !inside;
        }
    }
    
    return inside;
}

// Find closest point on a polygon edge
Vector closestPointOnPolygon(const Vector& point, const std::vector<Vector>& polygon) {
    double minDistSq = std::numeric_limits<double>::max();
    Vector closest;
    
    int nVerts = polygon.size();
    
    for (int i = 0; i < nVerts; i++) {
        int j = (i + 1) % nVerts;
        
        Vector edge = polygon[j] - polygon[i];
        double edgeLengthSq = edge.dotProduct(edge);
        
        if (edgeLengthSq > 0) {
            double t = std::clamp((point - polygon[i]).dotProduct(edge) / edgeLengthSq, 0.0, 1.0);
            Vector projected = polygon[i] + edge * t;
            
            double distSq = (point - projected).dotProduct(point - projected);
            if (distSq < minDistSq) {
                minDistSq = distSq;
                closest = projected;
            }
        }
    }
    
    return closest;
}

// Calculate the normal pointing OUT from a polygon at a given boundary point
Vector calculateOutwardNormal(const Vector& point, const Vector& boundaryPoint, 
                             const std::vector<Vector>& polygon) {
    // For a convex polygon, the vector from boundary to interior point is inward
    // So the outward normal is in the opposite direction
    Vector inward = point - boundaryPoint;
    double length = inward.length();
    
    if (length < 1e-6) {
        // If the point is exactly on the boundary, find the nearest edge and use its normal
        int nVerts = polygon.size();
        for (int i = 0; i < nVerts; i++) {
            int j = (i + 1) % nVerts;
            
            Vector edge = polygon[j] - polygon[i];
            double edgeLengthSq = edge.dotProduct(edge);
            
            if (edgeLengthSq > 0) {
                Vector toPoint = boundaryPoint - polygon[i];
                double proj = toPoint.dotProduct(edge) / edgeLengthSq;
                
                if (proj >= 0 && proj <= 1.0) {
                    // This edge is the closest
                    Vector edgeNormal(-edge.y, edge.x); // Perpendicular to edge
                    
                    // Make sure it points outward by checking if a point outside is on that side
                    Vector testPoint = boundaryPoint + edgeNormal * 0.01;
                    if (isPointInPolygon(testPoint, polygon)) {
                        edgeNormal = edgeNormal * -1.0; // Flip if it points inward
                    }
                    
                    return edgeNormal.normalized();
                }
            }
        }
        
        // Fallback
        return Vector(0, -1);
    }
    
    // Return normalized outward normal (opposite of inward)
    return (inward * -1.0) / length;
}

// Main update function
void RigidFluidPositionSolver::update(entt::registry& registry) {
    PROFILE_SCOPE("RigidFluidPositionSolver::update");
    
    // Collect entities
    auto fluidParticles = collectFluidParticles(registry);
    auto rigidBodies = collectRigidBodies(registry);
    
    // Early exit if no work
    if (fluidParticles.empty() || rigidBodies.empty()) {
        return;
    }
    
    // Process each fluid particle
    for (auto& fluid : fluidParticles) {
        Vector fluidPos(*fluid.pos);
        Vector totalCorrection(0, 0);
        bool needsCorrection = false;
        
        // Check against each rigid body
        for (const auto& rigid : rigidBodies) {
            // Circle collision
            if (rigid.shape.type == Components::ShapeType::Circle) {
                double distance = (fluidPos - Vector(rigid.pos)).length();
                double minDistance = rigid.shape.size + fluid.radius;
                
                if (distance < minDistance) {
                    // Calculate push direction (away from circle center)
                    Vector pushDir = fluidPos - Vector(rigid.pos);
                    
                    if (pushDir.length() < 1e-6) {
                        // If exactly at center, push in a default direction
                        pushDir = Vector(0, -1);
                    } else {
                        pushDir = pushDir.normalized();
                    }
                    
                    // Calculate push distance
                    double pushDistance = minDistance - distance + FluidRigidConfig::SafetyMargin;
                    pushDistance = std::max(pushDistance, FluidRigidConfig::MinPushDistance);
                    
                    totalCorrection += pushDir * pushDistance;
                    needsCorrection = true;
                }
            }
            // Polygon collision
            else if (rigid.shape.type == Components::ShapeType::Polygon && rigid.polygon) {
                // Transform polygon to world space
                auto worldPoly = getWorldSpacePolygon(*rigid.polygon, Vector(rigid.pos), rigid.angPos.angle);
                
                // Check if fluid particle is inside polygon
                if (isPointInPolygon(fluidPos, worldPoly)) {
                    // Find closest point on polygon boundary
                    Vector closestBoundary = closestPointOnPolygon(fluidPos, worldPoly);
                    
                    // Calculate outward normal and push distance
                    Vector pushDir = calculateOutwardNormal(fluidPos, closestBoundary, worldPoly);
                    double pushDistance = (closestBoundary - fluidPos).length() + 
                                       FluidRigidConfig::SafetyMargin + fluid.radius;
                    
                    // Ensure minimum push
                    pushDistance = std::max(pushDistance, FluidRigidConfig::MinPushDistance);
                    
                    totalCorrection += pushDir * pushDistance;
                    needsCorrection = true;
                }
            }
        }
        
        // Apply correction if needed
        if (needsCorrection) {
            fluid.pos->x += totalCorrection.x;
            fluid.pos->y += totalCorrection.y;
        }
    }
}

}  // namespace Systems
