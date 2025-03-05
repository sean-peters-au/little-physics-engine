/**
 * @fileoverview renderer_wasm.hpp
 * @brief Emscripten-based rendering for HTML5 Canvas or WebGL
 *
 * This minimal example uses the HTML5 canvas "2D" context via Emscripten's
 * C APIs. We draw basic circles/polygons for debugging.
 */

#pragma once

#include <entt/entt.hpp>
#include <string>

/**
 * @class WASMRenderer
 * @brief Renders the simulation to an HTML canvas using Emscripten calls
 */
class WASMRenderer
{
public:
    /**
     * @brief Constructor
     * @param width The canvas width in pixels
     * @param height The canvas height in pixels
     */
    WASMRenderer(int width, int height);

    /**
     * @brief Initialize the canvas context (2D or WebGL).
     * @return True on success, false otherwise
     */
    bool init();

    /**
     * @brief Clear the screen
     */
    void clear();

    /**
     * @brief Present the screen (noop with 2D canvas, but might flush in WebGL).
     */
    void present();

    /**
     * @brief Renders all particles from the ECS
     */
    void renderParticles(const entt::registry &registry);

    /**
     * @brief Renders minimal UI or debug info
     */
    void renderUI(const entt::registry &registry, bool paused);

private:
    int canvasWidth;
    int canvasHeight;
    bool initialized;
};