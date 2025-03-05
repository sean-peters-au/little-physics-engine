/**
 * @fileoverview main_wasm.cpp
 * @brief Emscripten entry point for the web-based N-body simulation.
 *
 * - Sets up the ECS simulator
 * - Builds the default scenario
 * - Runs the main loop via emscripten_set_main_loop
 * - Exports C++ functions so JavaScript can set scenario, pause, etc.
 */

#include <emscripten/emscripten.h>
#include <emscripten/html5.h>
#include <vector>
#include <chrono>

#include "sim.hpp"
#include "core/constants.hpp"
#include "entities/sim_components.hpp"
#include "core/profile.hpp"
#include "arch/wasm/renderer_wasm.hpp"

// Globals
static ECSSimulator* g_simulator = nullptr;
static WASMRenderer* g_renderer  = nullptr;
static bool g_paused = false;
static bool g_stepFrame = false;
static bool g_debug = false;

/**
 * @brief Helper: build a scenario list from getAllScenarios()
 *        Return as a static vector so we can index them from JS if we like.
 */
static std::vector<std::pair<SimulatorConstants::SimulationType, std::string>> g_scenarios;

static void buildScenarioList() {
    auto all = SimulatorConstants::getAllScenarios();
    for (auto s : all) {
        g_scenarios.push_back({ s, SimulatorConstants::getScenarioName(s) });
    }
}

/**
 * @brief Re-initialize the simulator with a new scenario
 *
 * @param scenario The scenario ID (from the enum)
 */
static void updateSimulatorState(ECSSimulator& simulator,
                                 SimulatorConstants::SimulationType scenario)
{
    SimulatorConstants::initializeConstants(scenario);
    simulator.setScenario(scenario);

    auto& registry = simulator.getRegistry();
    auto stView = registry.view<Components::SimulatorState>();
    if (!stView.empty()) {
        auto& st = registry.get<Components::SimulatorState>(stView.front());
        st.baseTimeAcceleration = SimulatorConstants::TimeAcceleration;
    }
    simulator.reset();
}

/**
 * @brief The main loop function called each frame by Emscripten.
 */
static void mainLoopIteration()
{
    // Step if not paused or stepping
    if (!g_paused || g_stepFrame) {
        g_simulator->tick();
        g_stepFrame = false;
    }

    // Clear, render
    g_renderer->clear();
    g_renderer->renderParticles(g_simulator->getRegistry());
    g_renderer->present();

    // Optionally, every N seconds, print stats
    static auto lastProfile = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - lastProfile).count();
    if (elapsed >= 10) {
        Profiling::Profiler::printStats();
        Profiling::Profiler::reset();
        lastProfile = now;
    }
}

/**
 * @brief Exports a function to toggle pause from JavaScript
 */
extern "C" EMSCRIPTEN_KEEPALIVE
void setPaused(int pausedFlag)
{
    g_paused = (pausedFlag != 0);
}

/**
 * @brief Exports a function to step one frame
 */
extern "C" EMSCRIPTEN_KEEPALIVE
void stepOneFrame()
{
    g_stepFrame = true;
}

/**
 * @brief Exports a function to reset the simulator (with the same scenario)
 */
extern "C" EMSCRIPTEN_KEEPALIVE
void resetSimulator()
{
    if (g_simulator) {
        g_simulator->reset();
        g_paused = false;
    }
}

/**
 * @brief Exports a function to set the debug-visualization flag
 */
extern "C" EMSCRIPTEN_KEEPALIVE
void setDebug(int debugFlag)
{
    g_debug = (debugFlag != 0);
}

/**
 * @brief Exports a function to set scenario by index in g_scenarios
 */
extern "C" EMSCRIPTEN_KEEPALIVE
void setScenarioByIndex(int index)
{
    if (index < 0 || index >= (int)g_scenarios.size()) {
        return;
    }
    auto scenarioEnum = g_scenarios[index].first;
    updateSimulatorState(*g_simulator, scenarioEnum);
    g_paused = false;
}

/**
 * @brief Exports a function to set playback speed (timeScale)
 */
extern "C" EMSCRIPTEN_KEEPALIVE
void setPlaybackSpeed(float factor)
{
    auto& registry = g_simulator->getRegistry();
    auto stView = registry.view<Components::SimulatorState>();
    if (!stView.empty()) {
        auto& st = registry.get<Components::SimulatorState>(stView.front());
        st.timeScale = factor;
    }
}

/**
 * @brief Emscripten main
 */
int main(int /*argc*/, char** /*argv*/)
{
    // Build scenario list
    buildScenarioList();

    // Create the simulator
    static CoordinateSystem coord;
    static ECSSimulator simulator(&coord);
    g_simulator = &simulator;

    // Default scenario
    updateSimulatorState(simulator, SimulatorConstants::SimulationType::KEPLERIAN_DISK);
    simulator.init();

    // Create a WASM-based renderer
    static WASMRenderer renderer(
        (int)SimulatorConstants::ScreenLength + 200,
        (int)SimulatorConstants::ScreenLength
    );
    if (!renderer.init()) {
        return 1;
    }
    g_renderer = &renderer;

    // Start main loop
    //  0 => use requestAnimationFrame
    emscripten_set_main_loop(mainLoopIteration, 0, 1);

    return 0; // Not reached
}