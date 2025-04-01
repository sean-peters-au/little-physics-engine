/**
 * @file renderer_types.hpp
 * @brief Shared types used across rendering components.
 */
#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <utility> // For std::pair
#include <vector>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/Rect.hpp>

// Include necessary component/constant definitions
#include "core/constants.hpp"
#include "entities/entity_components.hpp"

/**
 * @brief Hash function for pixel coordinates (copied from renderer_native.hpp)
 */
struct PixelCoordHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        std::size_t h1 = std::hash<int>()(p.first);
        std::size_t h2 = std::hash<int>()(p.second);
        return h1 ^ (h2 + 0x9e3779b97f4a7c16ULL + (h1 << 6) + (h1 >> 2));
    }
};

/**
 * @brief Stores aggregated physical properties for pixel-based effects/coloring.
 * (Moved from PresentationManager)
 */
struct PixelProperties {
    double density = 0.0;
    double temperature = 0.0;
    double total_mass = 0.0;
    int particle_count = 0;
    bool is_asleep = false;
    bool has_temperature = false;

    void add(const Components::Density* dens,
             const Components::Temperature* temp,
             const Components::Mass* mass,
             const Components::Sleep* sleep)
    {
        // Copied directly from Renderer::PixelProperties::add
        if (dens) {
            density += dens->value;
        }
        if (temp) {
            if (mass && total_mass + mass->value > 1e-9) { // Avoid division by zero/very small mass
                double w = mass->value;
                temperature = (temperature * total_mass + temp->value * w) / (total_mass + w);
            } else if (particle_count > 0) { // Avoid division by zero if no mass but existing particles
                temperature = (temperature * particle_count + temp->value) / (particle_count + 1);
            } else {
                 temperature = temp->value; // First particle
            }
            has_temperature = true; // Mark temperature as present
        }
        if (mass) {
            total_mass += mass->value;
        }
        if (sleep) {
            is_asleep = sleep->asleep;
        }
        particle_count++;
    }
};

/** Function type for mapping pixel properties to an sf::Color. */
using ColorMapper = std::function<sf::Color(const PixelProperties&)>;

// Add an enum for button identification
enum class ButtonID {
    UNKNOWN,
    PAUSE_PLAY,
    NEXT_FRAME,
    RESET,
    DEBUG_TOGGLE,
    // Speed buttons (can use multiplier directly, or add IDs)
    SPEED_0_25X,
    SPEED_0_5X,
    SPEED_1X,
    // Color scheme buttons
    COLOR_DEFAULT,
    COLOR_SLEEP,
    COLOR_TEMP,
    // Scenario buttons (can use scenario type directly, or add IDs)
    SCENARIO_BUTTON // Use scenario type stored in UIButton instead?
};

/** @brief UI button definition (Moved from PresentationManager) */
struct UIButton {
    ButtonID id = ButtonID::UNKNOWN; // Added ID
    sf::IntRect rect;
    std::string label;
    bool isSpecialButton;
    SimulatorConstants::SimulationType scenario; // Keep for scenario buttons
    double speedMultiplier; // Keep for speed buttons

    UIButton()
        : rect()
        , label()
        , isSpecialButton(false)
        , scenario(SimulatorConstants::SimulationType::KEPLERIAN_DISK) // Default
        , speedMultiplier(1.0)
    {}
};

/** @brief Color scheme options (Moved from PresentationManager) */
enum class ColorScheme {
    DEFAULT,
    SLEEP,
    TEMPERATURE
}; 