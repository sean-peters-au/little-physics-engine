/**
 * @fileoverview renderer_wasm.cpp
 * @brief Implementation of a minimal Emscripten-based renderer using the HTML5 canvas.
 *
 * This is a stripped-down example that:
 * - Obtains a "2D" context from a <canvas> element
 * - Uses emscripten's HTML5 APIs to draw circles and polygons
 * - Ignores fancy UI (we assume JS code draws UI over the same canvas or in HTML)
 */

#include <cmath>
#include <string>
#include <iostream>

#include <emscripten/html5.h>

#include "nbody/arch/wasm/renderer_wasm.hpp"
#include "nbody/core/constants.hpp"
#include "nbody/components/basic.hpp"
#include "nbody/math/polygon.hpp"

WASMRenderer::WASMRenderer(int width, int height)
    : canvasWidth(width)
    , canvasHeight(height)
    , initialized(false)
{
}

bool WASMRenderer::init()
{
    // Attempt to set the size of the canvas
    // Emscripten tries to find an element with id="canvas"
    emscripten_set_canvas_element_size("#canvas", canvasWidth, canvasHeight);

    // Optionally check if we have a valid context
    // For pure 2D: 'EMSCRIPTEN_WEBGL_CONTEXT_HANDLE' isn't needed.
    // We'll just assume success for this minimal example.
    initialized = true;
    return true;
}

void WASMRenderer::clear()
{
    // We can call emscripten_run_script to run JS: "ctx.clearRect(0,0, w,h);" or use C API
    std::string script = "var ctx = document.getElementById('canvas').getContext('2d');"
                         "ctx.fillStyle='#000000';"  // black
                         "ctx.fillRect(0,0," + std::to_string(canvasWidth) + "," + std::to_string(canvasHeight) + ");";
    emscripten_run_script(script.c_str());
}

void WASMRenderer::present()
{
    // For 2D canvas, there's nothing to "swap". For WebGL, you might swap buffers here.
}

void WASMRenderer::renderParticles(const entt::registry &registry)
{
    // We'll do *very* basic circle drawing
    // For each entity with Position & maybe shape, we do a "ctx.beginPath() ... ctx.arc() ..."

    auto view = registry.view<Components::Position, Components::Shape>();
    for (auto e : view) {
        auto &pos = view.get<Components::Position>(e);
        auto &shape = view.get<Components::Shape>(e);

        float px = (float)SimulatorConstants::metersToPixels(pos.x);
        float py = (float)SimulatorConstants::metersToPixels(pos.y);

        // For now, just draw a circle ignoring shape.type
        float r = (float)std::max(1.0, SimulatorConstants::metersToPixels(shape.size));
        // We'll create a small JS string that draws a circle at (px,py).
        // (In a real engine, you'd batch these into one big script.)
        std::string script = "var ctx=document.getElementById('canvas').getContext('2d');"
                             "ctx.beginPath();"
                             "ctx.arc(" + std::to_string(px) + "," + std::to_string(py) + ","
                                          + std::to_string(r)  + ",0,6.283185);"
                             "ctx.fillStyle='#FFFFFF';"
                             "ctx.fill();";
        emscripten_run_script(script.c_str());
    }
}

void WASMRenderer::renderUI(const entt::registry &registry, bool paused)
{
    // Minimal text overlay to indicate paused or not
    // We'll just place it in the top-left corner with a small font
    std::string text = paused ? "Paused" : "Running";
    std::string script =
      "var ctx=document.getElementById('canvas').getContext('2d');"
      "ctx.fillStyle='#FFFF00';"
      "ctx.font='16px sans-serif';"
      "ctx.fillText('" + text + "',10,20);";
    emscripten_run_script(script.c_str());
}