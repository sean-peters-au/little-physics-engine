/**
 * @fileoverview main_native.cpp
 * @brief Main entry point for the native (non-WASM) application.
 *
 * Initializes the SimManager singleton and runs the main simulation loop.
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
    SimManager::getInstance().run();

    return 0;
}
