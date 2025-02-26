/**
 * @file utils.cpp
 * @brief Implementation of shared utilities for rigid-fluid interaction systems
 */

#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

#include <entt/entt.hpp>

#include "nbody/math/vector_math.hpp"
#include "nbody/math/polygon.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/components/sph.hpp"
#include "nbody/systems/rigid_fluid/utils.hpp"

namespace Systems {
namespace RigidFluidUtils {

// Collect all fluid particles with common properties
std::vector<FluidParticle> collectFluidParticles(entt::registry& registry) {
    std::vector<FluidParticle> particles;
    
    auto view = registry.view<Components::ParticlePhase, 
                             Components::Position,
                             Components::Velocity,
                             Components::Mass,
                             Components::SmoothingLength>();
    
    for (auto entity : view) {
        const auto& phase = view.get<Components::ParticlePhase>(entity);
        if (phase.phase == Components::Phase::Liquid) {
            FluidParticle particle;
            particle.id = entity;
            particle.pos = view.get<Components::Position>(entity);
            particle.vel = view.get<Components::Velocity>(entity);
            particle.mass = view.get<Components::Mass>(entity).value;
            particle.smoothingLength = view.get<Components::SmoothingLength>(entity).value;
            particle.radius = particle.smoothingLength * 0.5; // Half smoothing length as radius
            
            // Get density if available
            if (auto* sphTemp = registry.try_get<Components::SPHTemp>(entity)) {
                particle.density = sphTemp->density;
            } else {
                particle.density = 1000.0; // Default density
            }
            
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
                             Components::Velocity,
                             Components::Shape,
                             Components::AngularPosition,
                             Components::AngularVelocity,
                             Components::Mass,
                             Components::Inertia>();
    
    for (auto entity : view) {
        const auto& phase = view.get<Components::ParticlePhase>(entity);
        if (phase.phase == Components::Phase::Solid) {
            RigidBody body;
            body.id = entity;
            body.pos = &view.get<Components::Position>(entity);
            body.vel = &view.get<Components::Velocity>(entity);
            body.angPos = &view.get<Components::AngularPosition>(entity);
            body.angVel = &view.get<Components::AngularVelocity>(entity);
            body.mass = view.get<Components::Mass>(entity);
            body.inertia = view.get<Components::Inertia>(entity);
            body.shape = view.get<Components::Shape>(entity);
            
            // Get polygon shape if applicable
            if (body.shape.type == Components::ShapeType::Polygon) {
                body.polygon = registry.try_get<PolygonShape>(entity);
            } else {
                body.polygon = nullptr;
            }
            
            // Calculate AABB
            double margin = body.shape.size * 1.5;
            body.minX = body.pos->x - margin;
            body.minY = body.pos->y - margin;
            body.maxX = body.pos->x + margin;
            body.maxY = body.pos->y + margin;
            
            // Refine AABB for polygons
            if (body.polygon && body.shape.type == Components::ShapeType::Polygon) {
                body.minX = body.maxX = body.pos->x;
                body.minY = body.maxY = body.pos->y;
                
                for (const auto& v : body.polygon->vertices) {
                    Vector rotated = v.rotateByAngle(body.angPos->angle);
                    double vx = body.pos->x + rotated.x;
                    double vy = body.pos->y + rotated.y;
                    
                    body.minX = std::min(body.minX, vx);
                    body.minY = std::min(body.minY, vy);
                    body.maxX = std::max(body.maxX, vx);
                    body.maxY = std::max(body.maxY, vy);
                }
                
                // Add margin
                double safetyMargin = 0.1;
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
        // Check if point is on an edge or vertex
        Vector edge = polygon[i] - polygon[j];
        double edgeLengthSq = edge.dotProduct(edge);
        if (edgeLengthSq > 0) {
            Vector pointToJ = point - polygon[j];
            double t = std::clamp(pointToJ.dotProduct(edge) / edgeLengthSq, 0.0, 1.0);
            Vector projected = polygon[j] + edge * t;
            double distSq = (point - projected).dotProduct(point - projected);
            
            if (distSq < 1e-10) {
                return true; // Point is on an edge
            }
        }
        
        // Ray casting algorithm
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

// Calculate normal pointing out from a polygon
Vector calculateOutwardNormal(const Vector& point, const Vector& boundaryPoint, 
                             const std::vector<Vector>& polygon) {
    // Vector from boundary to interior point is inward
    Vector inward = point - boundaryPoint;
    double length = inward.length();
    
    if (length < 1e-6) {
        // If point is exactly on boundary, find nearest edge and use its normal
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
                    
                    // Make sure it points outward
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

} // namespace RigidFluidUtils
} // namespace Systems 