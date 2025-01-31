/**
 * @fileoverview main_wasm.cpp
 * @brief Emscripten entry point for running the simulation in a web browser.
 *
 * Replaces the SFML loop with emscripten_set_main_loop, pumping the simulation
 * and drawing via our WASM-compatible renderer.
 */

#include <chrono>
#include <vector>

#include <emscripten/emscripten.h>
#include <emscripten/html5.h>

#include "nbody/core/simulator.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/arch/wasm/renderer_wasm.hpp"
#include "nbody/components/sim.hpp"
#include "nbody/core/profile.hpp"

// Global or static references
static ECSSimulator *g_simulator = nullptr;
static WASMRenderer *g_renderer = nullptr;
static bool g_paused = false;
static bool g_running = true;
static bool g_stepFrame = false;

/**
 * @brief For scenario selection / re-init
 */
static void updateSimulatorState(ECSSimulator& simulator,
                                 SimulatorConstants::SimulationType scenario)
{
    // Same logic as before
    SimulatorConstants::initializeConstants(scenario);
    simulator.setScenario(scenario);

    auto& registry = simulator.getRegistry();
    auto stateView = registry.view<Components::SimulatorState>();
    if (!stateView.empty()) {
        auto& state = registry.get<Components::SimulatorState>(stateView.front());
        state.baseTimeAcceleration = SimulatorConstants::TimeAcceleration;
    }
    simulator.reset();
}

/**
 * @brief Called repeatedly by Emscripten's main loop
 */
static void mainLoopIteration()
{
    // We'll do basic event dispatch from JS side, or input checks
    // For now, we skip event handling or do a basic check if user wants to pause:
    if (!g_paused || g_stepFrame) {
        g_simulator->tick();
        g_stepFrame = false;
    }

    g_renderer->clear();
    g_renderer->renderParticles(g_simulator->getRegistry());
    g_renderer->renderUI(g_simulator->getRegistry(), g_paused);
    g_renderer->present();

    // Optionally print & reset profiler every 10s?
    // (In WASM, a high-level timer can do this too.)
}

/**
 * @brief Emscripten entry point.
 * Note: 'main' is still used, but we do not do a while loop.
 */
int main(int argc, char **argv)
{
    // Create the simulator
    static CoordinateSystem coordSystem;
    static ECSSimulator simulator(&coordSystem);
    g_simulator = &simulator;

    // Initialize with a default scenario
    updateSimulatorState(simulator, SimulatorConstants::SimulationType::KEPLERIAN_DISK);
    simulator.init();

    // Create our WASM-based renderer
    static WASMRenderer renderer((int)SimulatorConstants::ScreenLength + 200,
                                 (int)SimulatorConstants::ScreenLength);
    if (!renderer.init()) {
        // If fails, just return
        return 1;
    }
    g_renderer = &renderer;

    // Start main loop via Emscripten
    const int fps = 60; // or 0 for 'requestAnimationFrame'
    emscripten_set_main_loop(mainLoopIteration, fps, /*simulate_infinite_loop=*/1);

    return 0; // never actually returns
}

// You can add JS-exposed functions for pause, scenario switching, etc. via EMSCRIPTEN_BINDINGS