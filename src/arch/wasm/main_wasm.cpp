/**
 * @brief Emscripten entry point for the web-based physics simulation.
 */

#include <emscripten/emscripten.h>
#include <emscripten/html5.h>

/**
 * @brief Emscripten main
 */
int main(int /*argc*/, char** /*argv*/)
{
    /*
     * I've deleted all this code after moving to a metal based fluid system. I'd need to
     * rewrite to use a webgpu based fluid system. And I can't be bothered to do that right now.
     */

    return 0; // Not reached
}