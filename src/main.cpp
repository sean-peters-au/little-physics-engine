#include "ecs_simulator.h"
#include "simulator_constants.h"
#include "systems/barnes_hut_system.h"
#include "renderer.h"

#include <iostream>

int main(int, char**) {
	// Initialize simulation constants for gas-to-planet formation
	SimulatorConstants::initializeConstants(SimulatorConstants::SimulationType::CELESTIAL_GAS);

	// Create and initialize the renderer
	Renderer renderer(SimulatorConstants::ScreenLength, SimulatorConstants::ScreenLength);
	if (!renderer.init()) {
		return 1;
	}

	CoordinateSystem coordSystem;
	ECSSimulator simulator(&coordSystem);
	simulator.init();

	const int FPS = 60;
	const int frameDelay = 1000 / FPS;
	Uint32 frameStart;
	int frameTime;
	float avgFPS = 0;
	int frameCount = 0;
	Uint32 fpsTimer = SDL_GetTicks();

	bool running = true;
	while(running) {
		frameStart = SDL_GetTicks();

		SDL_Event event;
		while(SDL_PollEvent(&event)) {
			if(event.type == SDL_QUIT) {
				running = false;
			}
		}

		simulator.tick();
		
		renderer.clear();
		renderer.renderParticles(simulator.getRegistry(), Renderer::whiteColor);
		
		// Update and render FPS
		frameTime = SDL_GetTicks() - frameStart;
		frameCount++;
		
		if (SDL_GetTicks() - fpsTimer >= 1000) {
			avgFPS = frameCount / ((SDL_GetTicks() - fpsTimer) / 1000.0f);
			renderer.renderFPS(avgFPS);
			
			frameCount = 0;
			fpsTimer = SDL_GetTicks();
		}
		
		renderer.present();
		
		if (frameDelay > frameTime) {
			SDL_Delay(frameDelay - frameTime);
		}
	}

	return 0;
}
