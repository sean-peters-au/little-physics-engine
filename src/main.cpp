#include "ecs_simulator.h"
#include "simulator_constants.h"
#include "systems/barnes_hut_system.h"
#include "renderer.h"

#include <iostream>

int main(int, char**) {
    // Start with CELESTIAL_GAS scenario
    SimulatorConstants::initializeConstants(SimulatorConstants::SimulationType::CELESTIAL_GAS);

    Renderer renderer(SimulatorConstants::ScreenLength, SimulatorConstants::ScreenLength);
    if (!renderer.init()) {
        return 1;
    }

    CoordinateSystem coordSystem;
    ECSSimulator simulator(&coordSystem);
    simulator.setScenario(SimulatorConstants::SimulationType::CELESTIAL_GAS);
    simulator.init();

    const int FPS = 60;
    const int frameDelay = 1000 / FPS;
    Uint32 frameStart;
    int frameTime;
    float avgFPS = 0;
    int frameCount = 0;
    Uint32 fpsTimer = SDL_GetTicks();

    bool running = true;
    bool paused = false;

    while(running) {
        frameStart = SDL_GetTicks();

        SDL_Event event;
        while(SDL_PollEvent(&event)) {
            if(event.type == SDL_QUIT) {
                running = false;
            } else if (event.type == SDL_KEYDOWN) {
                switch (event.key.keysym.sym) {
                    case SDLK_p:
                        paused = !paused;
                        break;
                    case SDLK_r:
                        simulator.reset();
                        paused = false;
                        break;
                    case SDLK_1:
                        simulator.setScenario(SimulatorConstants::SimulationType::CELESTIAL_GAS);
                        simulator.reset();
                        paused = false;
                        break;
                    case SDLK_2:
                        simulator.setScenario(SimulatorConstants::SimulationType::ISOTHERMAL_BOX);
                        simulator.reset();
                        paused = false;
                        break;
                    default:
                        break;
                }
            }
        }

        if (!paused) {
            simulator.tick();
        }

        renderer.clear();
        renderer.renderParticles(simulator.getRegistry(), Renderer::whiteColor);

        // Update and render FPS
        frameTime = SDL_GetTicks() - frameStart;
        frameCount++;

        if (SDL_GetTicks() - fpsTimer >= 1000) {
            avgFPS = frameCount / ((SDL_GetTicks() - fpsTimer) / 1000.0f);
            frameCount = 0;
            fpsTimer = SDL_GetTicks();
        }

        renderer.renderFPS(avgFPS);

        // Render UI overlay
        renderer.renderUI(paused, simulator.getCurrentScenario());

        renderer.present();

        if (frameDelay > frameTime) {
            SDL_Delay(frameDelay - frameTime);
        }
    }

    return 0;
}