/**
 * @file main_native.cpp
 * @brief Main entry point for the native platform.
 *
 * This file creates a SimManager instance, initializes it, and runs the main loop.
 */

#define NS_PRIVATE_IMPLEMENTATION
#define CA_PRIVATE_IMPLEMENTATION
#define MTL_PRIVATE_IMPLEMENTATION

#include <Foundation/Foundation.hpp>
#include <Metal/Metal.hpp>
#include <QuartzCore/QuartzCore.hpp>

#include <chrono>
#include <SFML/System.hpp>
#include <iostream>

#include "core/profile.hpp"
#include "sim_manager.hpp"

int main() {
    // Profile the entire application run
    PROFILE_SCOPE("main");

    // Get the SimManager singleton instance
    SimManager& simManager = SimManager::getInstance();

    // Run the simulation loop via the singleton
    simManager.run();

    return 0;
}
